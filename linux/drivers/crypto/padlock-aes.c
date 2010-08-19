/* 
 * Cryptographic API.
 *
 * Support for VIA PadLock hardware crypto engine.
 *
 * Copyright (c) 2004  Michal Ludvig <michal@logix.cz>
 *
 */

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <asm/byteorder.h>
#include "padlock.h"

/* Control word. */
struct cword {
	unsigned int __attribute__ ((__packed__))
		rounds:4,
		algo:3,
		keygen:1,
		interm:1,
		encdec:1,
		ksize:2;
} __attribute__ ((__aligned__(PADLOCK_ALIGNMENT)));

/* Whenever making any changes to the following
 * structure *make sure* you keep E, d_data
 * and cword aligned on 16 Bytes boundaries and
 * the Hardware can access 16 * 16 bytes of E and d_data
 * (only the first 15 * 16 bytes matter but the HW reads
 * more).
 */
struct aes_ctx {
	u32 E[AES_MAX_KEYLENGTH_U32]
		__attribute__ ((__aligned__(PADLOCK_ALIGNMENT)));
	u32 d_data[AES_MAX_KEYLENGTH_U32]
		__attribute__ ((__aligned__(PADLOCK_ALIGNMENT)));
	struct {
		struct cword encrypt;
		struct cword decrypt;
	} cword;
	u32 *D;
};

/* Tells whether the ACE is capable to generate
   the extended key for a given key_len. */
static inline int
aes_hw_extkey_available(uint8_t key_len)
{
	/* TODO: We should check the actual CPU model/stepping
	         as it's possible that the capability will be
	         added in the next CPU revisions. */
	if (key_len == 16)
		return 1;
	return 0;
}

static inline struct aes_ctx *aes_ctx_common(void *ctx)
{
	unsigned long addr = (unsigned long)ctx;
	unsigned long align = PADLOCK_ALIGNMENT;

	if (align <= crypto_tfm_ctx_alignment())
		align = 1;
	return (struct aes_ctx *)ALIGN(addr, align);
}

static inline struct aes_ctx *aes_ctx(struct crypto_tfm *tfm)
{
	return aes_ctx_common(crypto_tfm_ctx(tfm));
}

static inline struct aes_ctx *blk_aes_ctx(struct crypto_blkcipher *tfm)
{
	return aes_ctx_common(crypto_blkcipher_ctx(tfm));
}

static int aes_set_key(struct crypto_tfm *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	struct aes_ctx *ctx = aes_ctx(tfm);
	const __le32 *key = (const __le32 *)in_key;
	u32 *flags = &tfm->crt_flags;
	struct crypto_aes_ctx gen_aes;

	if (key_len % 8) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	/*
	 * If the hardware is capable of generating the extended key
	 * itself we must supply the plain key for both encryption
	 * and decryption.
	 */
	ctx->D = ctx->E;

	ctx->E[0] = le32_to_cpu(key[0]);
	ctx->E[1] = le32_to_cpu(key[1]);
	ctx->E[2] = le32_to_cpu(key[2]);
	ctx->E[3] = le32_to_cpu(key[3]);

	/* Prepare control words. */
	memset(&ctx->cword, 0, sizeof(ctx->cword));

	ctx->cword.decrypt.encdec = 1;
	ctx->cword.encrypt.rounds = 10 + (key_len - 16) / 4;
	ctx->cword.decrypt.rounds = ctx->cword.encrypt.rounds;
	ctx->cword.encrypt.ksize = (key_len - 16) / 8;
	ctx->cword.decrypt.ksize = ctx->cword.encrypt.ksize;

	/* Don't generate extended keys if the hardware can do it. */
	if (aes_hw_extkey_available(key_len))
		return 0;

	ctx->D = ctx->d_data;
	ctx->cword.encrypt.keygen = 1;
	ctx->cword.decrypt.keygen = 1;

	if (crypto_aes_expand_key(&gen_aes, in_key, key_len)) {
		*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	memcpy(ctx->E, gen_aes.key_enc, AES_MAX_KEYLENGTH);
	memcpy(ctx->D, gen_aes.key_dec, AES_MAX_KEYLENGTH);
	return 0;
}

/* ====== Encryption/decryption routines ====== */

/* These are the real call to PadLock. */
static inline void padlock_reset_key(void)
{
	asm volatile ("pushfl; popfl");
}

static inline void padlock_xcrypt(const u8 *input, u8 *output, void *key,
				  void *control_word)
{
	asm volatile (".byte 0xf3,0x0f,0xa7,0xc8"	/* rep xcryptecb */
		      : "+S"(input), "+D"(output)
		      : "d"(control_word), "b"(key), "c"(1));
}

static void aes_crypt_copy(const u8 *in, u8 *out, u32 *key, struct cword *cword)
{
	u8 buf[AES_BLOCK_SIZE * 2 + PADLOCK_ALIGNMENT - 1];
	u8 *tmp = PTR_ALIGN(&buf[0], PADLOCK_ALIGNMENT);

	memcpy(tmp, in, AES_BLOCK_SIZE);
	padlock_xcrypt(tmp, out, key, cword);
}

static inline void aes_crypt(const u8 *in, u8 *out, u32 *key,
			     struct cword *cword)
{
	/* padlock_xcrypt requires at least two blocks of data. */
	if (unlikely(!(((unsigned long)in ^ (PAGE_SIZE - AES_BLOCK_SIZE)) &
		       (PAGE_SIZE - 1)))) {
		aes_crypt_copy(in, out, key, cword);
		return;
	}

	padlock_xcrypt(in, out, key, cword);
}

static inline void padlock_xcrypt_ecb(const u8 *input, u8 *output, void *key,
				      void *control_word, u32 count)
{
	if (count == 1) {
		aes_crypt(input, output, key, control_word);
		return;
	}

	asm volatile ("test $1, %%cl;"
		      "je 1f;"
		      "lea -1(%%ecx), %%eax;"
		      "mov $1, %%ecx;"
		      ".byte 0xf3,0x0f,0xa7,0xc8;"	/* rep xcryptecb */
		      "mov %%eax, %%ecx;"
		      "1:"
		      ".byte 0xf3,0x0f,0xa7,0xc8"	/* rep xcryptecb */
		      : "+S"(input), "+D"(output)
		      : "d"(control_word), "b"(key), "c"(count)
		      : "ax");
}

static inline u8 *padlock_xcrypt_cbc(const u8 *input, u8 *output, void *key,
				     u8 *iv, void *control_word, u32 count)
{
	/* rep xcryptcbc */
	asm volatile (".byte 0xf3,0x0f,0xa7,0xd0"
		      : "+S" (input), "+D" (output), "+a" (iv)
		      : "d" (control_word), "b" (key), "c" (count));
	return iv;
}

static void aes_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct aes_ctx *ctx = aes_ctx(tfm);
	padlock_reset_key();
	aes_crypt(in, out, ctx->E, &ctx->cword.encrypt);
}

static void aes_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct aes_ctx *ctx = aes_ctx(tfm);
	padlock_reset_key();
	aes_crypt(in, out, ctx->D, &ctx->cword.decrypt);
}

static struct crypto_alg aes_alg = {
	.cra_name		=	"aes",
	.cra_driver_name	=	"aes-padlock",
	.cra_priority		=	PADLOCK_CRA_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct aes_ctx),
	.cra_alignmask		=	PADLOCK_ALIGNMENT - 1,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(aes_alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey	   	= 	aes_set_key,
			.cia_encrypt	 	=	aes_encrypt,
			.cia_decrypt	  	=	aes_decrypt,
		}
	}
};

static int ecb_aes_encrypt(struct blkcipher_desc *desc,
			   struct scatterlist *dst, struct scatterlist *src,
			   unsigned int nbytes)
{
	struct aes_ctx *ctx = blk_aes_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	padlock_reset_key();

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		padlock_xcrypt_ecb(walk.src.virt.addr, walk.dst.virt.addr,
				   ctx->E, &ctx->cword.encrypt,
				   nbytes / AES_BLOCK_SIZE);
		nbytes &= AES_BLOCK_SIZE - 1;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static int ecb_aes_decrypt(struct blkcipher_desc *desc,
			   struct scatterlist *dst, struct scatterlist *src,
			   unsigned int nbytes)
{
	struct aes_ctx *ctx = blk_aes_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	padlock_reset_key();

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		padlock_xcrypt_ecb(walk.src.virt.addr, walk.dst.virt.addr,
				   ctx->D, &ctx->cword.decrypt,
				   nbytes / AES_BLOCK_SIZE);
		nbytes &= AES_BLOCK_SIZE - 1;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static struct crypto_alg ecb_aes_alg = {
	.cra_name		=	"ecb(aes)",
	.cra_driver_name	=	"ecb-aes-padlock",
	.cra_priority		=	PADLOCK_COMPOSITE_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct aes_ctx),
	.cra_alignmask		=	PADLOCK_ALIGNMENT - 1,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(ecb_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	AES_MIN_KEY_SIZE,
			.max_keysize		=	AES_MAX_KEY_SIZE,
			.setkey	   		= 	aes_set_key,
			.encrypt		=	ecb_aes_encrypt,
			.decrypt		=	ecb_aes_decrypt,
		}
	}
};

static int cbc_aes_encrypt(struct blkcipher_desc *desc,
			   struct scatterlist *dst, struct scatterlist *src,
			   unsigned int nbytes)
{
	struct aes_ctx *ctx = blk_aes_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	padlock_reset_key();

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		u8 *iv = padlock_xcrypt_cbc(walk.src.virt.addr,
					    walk.dst.virt.addr, ctx->E,
					    walk.iv, &ctx->cword.encrypt,
					    nbytes / AES_BLOCK_SIZE);
		memcpy(walk.iv, iv, AES_BLOCK_SIZE);
		nbytes &= AES_BLOCK_SIZE - 1;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static int cbc_aes_decrypt(struct blkcipher_desc *desc,
			   struct scatterlist *dst, struct scatterlist *src,
			   unsigned int nbytes)
{
	struct aes_ctx *ctx = blk_aes_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	padlock_reset_key();

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		padlock_xcrypt_cbc(walk.src.virt.addr, walk.dst.virt.addr,
				   ctx->D, walk.iv, &ctx->cword.decrypt,
				   nbytes / AES_BLOCK_SIZE);
		nbytes &= AES_BLOCK_SIZE - 1;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static struct crypto_alg cbc_aes_alg = {
	.cra_name		=	"cbc(aes)",
	.cra_driver_name	=	"cbc-aes-padlock",
	.cra_priority		=	PADLOCK_COMPOSITE_PRIORITY,
	.cra_flags		=	CRYPTO_ALG_TYPE_BLKCIPHER,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct aes_ctx),
	.cra_alignmask		=	PADLOCK_ALIGNMENT - 1,
	.cra_type		=	&crypto_blkcipher_type,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(cbc_aes_alg.cra_list),
	.cra_u			=	{
		.blkcipher = {
			.min_keysize		=	AES_MIN_KEY_SIZE,
			.max_keysize		=	AES_MAX_KEY_SIZE,
			.ivsize			=	AES_BLOCK_SIZE,
			.setkey	   		= 	aes_set_key,
			.encrypt		=	cbc_aes_encrypt,
			.decrypt		=	cbc_aes_decrypt,
		}
	}
};

static int __init padlock_init(void)
{
	int ret;

	if (!cpu_has_xcrypt) {
		printk(KERN_ERR PFX "VIA PadLock not detected.\n");
		return -ENODEV;
	}

	if (!cpu_has_xcrypt_enabled) {
		printk(KERN_ERR PFX "VIA PadLock detected, but not enabled. Hmm, strange...\n");
		return -ENODEV;
	}

	if ((ret = crypto_register_alg(&aes_alg)))
		goto aes_err;

	if ((ret = crypto_register_alg(&ecb_aes_alg)))
		goto ecb_aes_err;

	if ((ret = crypto_register_alg(&cbc_aes_alg)))
		goto cbc_aes_err;

	printk(KERN_NOTICE PFX "Using VIA PadLock ACE for AES algorithm.\n");

out:
	return ret;

cbc_aes_err:
	crypto_unregister_alg(&ecb_aes_alg);
ecb_aes_err:
	crypto_unregister_alg(&aes_alg);
aes_err:
	printk(KERN_ERR PFX "VIA PadLock AES initialization failed.\n");
	goto out;
}

static void __exit padlock_fini(void)
{
	crypto_unregister_alg(&cbc_aes_alg);
	crypto_unregister_alg(&ecb_aes_alg);
	crypto_unregister_alg(&aes_alg);
}

module_init(padlock_init);
module_exit(padlock_fini);

MODULE_DESCRIPTION("VIA PadLock AES algorithm support");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal Ludvig");

MODULE_ALIAS("aes");
