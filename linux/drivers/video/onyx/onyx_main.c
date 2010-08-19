#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <video/bs_chip.h>
#include <video/bs_cmd.h>

#define ANIMATION_TIMER 100
#define ANIM_X            0
#define ANIM_HEIGHT      25
#define NUM_RECTS        10

extern int    screen_width;
extern int    screen_height;
extern int    wfm_mode;
static int    progress = 0;
static u8*    animation_buf = NULL;
static struct timer_list onyx_animation_timer;

static void update_area(u16 x, u16 y, u16 width, u16 height)
{
    wait_for_ready();
    bs_cmd_upd_part_area(1, 15, 0, x, y, width, height);
}

static void draw_outline(void)
{
    int i = 0;
    int anim_y = screen_height - ANIM_HEIGHT;

    // Make area all white.
    memset(animation_buf, 0xFF, screen_width * ANIM_HEIGHT);

    // Draw outside rectangle.
    memset(animation_buf, 0, screen_width * 2);
    memset(animation_buf + (ANIM_HEIGHT - 2) * screen_width, 0, screen_width * 2);
    for (i = 2; i < ANIM_HEIGHT - 2; i++)
    {
        animation_buf[i * screen_width] = 0;
        animation_buf[i * screen_width + 1] = 0;
        animation_buf[i * screen_width + screen_width - 2] = 0;
        animation_buf[i * screen_width + screen_width - 1] = 0;
    }

    // Update animation area.
    bs_cmd_ld_img_area_data(3, ANIM_X, anim_y, screen_width, ANIM_HEIGHT, animation_buf);
    update_area(ANIM_X, anim_y, screen_width, ANIM_HEIGHT);
}

static void animation(unsigned long param)
{
    u32 i = 0;
    u32 w = 0;
    int anim_y = screen_height - ANIM_HEIGHT;

    // Draw internal rectange and fill with black.
    w = (screen_width - 6) / NUM_RECTS - 2;
    for (i = 3; i < ANIM_HEIGHT - 3; i++)
    {
        if (progress == NUM_RECTS - 1)
        {
            u32 last_w = w + (screen_width - 6) % NUM_RECTS;
            memset(animation_buf + i * screen_width + 4 + (2 + w) * progress, 0, last_w);
        }
        else
        {
            memset(animation_buf + i * screen_width + 4 + (2 + w) * progress, 0, w);
        }
    }

    // Update animation area.
    bs_cmd_ld_img_area_data(3, ANIM_X, anim_y, screen_width, ANIM_HEIGHT, animation_buf);
    update_area(ANIM_X, anim_y, screen_width, ANIM_HEIGHT);

    if (++progress < NUM_RECTS)
    {
        // Update timer.
        mod_timer(&onyx_animation_timer, jiffies + ANIMATION_TIMER);
    }
    else
    {
        del_timer(&onyx_animation_timer);
        kfree(animation_buf);
    }
}

static int __init onyx_splash_init(void)
{
    printk("Onyx animation splash loaded.\n");

    init_timer(&onyx_animation_timer);
    onyx_animation_timer.expires = jiffies + ANIMATION_TIMER;
    onyx_animation_timer.function = animation;
    add_timer(&onyx_animation_timer);

    animation_buf = kmalloc(screen_width * ANIM_HEIGHT, GFP_KERNEL);
    if (animation_buf == NULL)
    {
        return -ENOMEM;
    }

    draw_outline();
    return 0;
}

static void onyx_splash_exit(void)
{
}

MODULE_LICENSE("GPL");
module_init(onyx_splash_init);
module_exit(onyx_splash_exit);
