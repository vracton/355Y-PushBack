/*******************************************************************************
 * Size: 12 px
 * Bpp: 1
 * Opts: --bpp 1 --size 12 --no-compress --stride 1 --align 1 --font Comic Sans MS (1).ttf --symbols abcdefghijklmnopqstuvwxyz --range 97-122 --format lvgl -o comic_sans.c
 ******************************************************************************/

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "../include/liblvgl/lvgl.h"
#else
    #include "../include/liblvgl/lvgl.h"
#endif



#ifndef COMIC_SANS
#define COMIC_SANS 1
#endif

#if COMIC_SANS

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0061 "a" */
    0x7b, 0x28, 0xa2, 0x89, 0xf0,

    /* U+0062 "b" */
    0x82, 0x8, 0x3e, 0xce, 0x18, 0x63, 0xf8,

    /* U+0063 "c" */
    0x3a, 0x61, 0x8, 0xbc,

    /* U+0064 "d" */
    0x4, 0x10, 0x5f, 0xc6, 0x18, 0x71, 0x7c,

    /* U+0065 "e" */
    0x7b, 0x2b, 0xb0, 0xc5, 0xe0,

    /* U+0066 "f" */
    0x1c, 0x82, 0x3e, 0x20, 0x82, 0x8, 0x20, 0x80,

    /* U+0067 "g" */
    0x7b, 0x28, 0xa2, 0x99, 0xa0, 0x86, 0xf0,

    /* U+0068 "h" */
    0x84, 0x21, 0x7c, 0xc6, 0x31, 0x88,

    /* U+0069 "i" */
    0xc2, 0xaa, 0x80,

    /* U+006A "j" */
    0x20, 0x1, 0x11, 0x11, 0x11, 0x57,

    /* U+006B "k" */
    0x84, 0x21, 0x1b, 0x73, 0xd3, 0x88,

    /* U+006C "l" */
    0xff, 0x80,

    /* U+006D "m" */
    0xbe, 0xd2, 0x92, 0x92, 0x8b, 0x89,

    /* U+006E "n" */
    0xbe, 0x63, 0x18, 0xc4,

    /* U+006F "o" */
    0x76, 0x63, 0x18, 0xb8,

    /* U+0070 "p" */
    0xf4, 0x63, 0x18, 0xfa, 0x10, 0x80,

    /* U+0071 "q" */
    0x3e, 0x63, 0x18, 0xb8, 0x42, 0x10,

    /* U+0072 "r" */
    0xf9, 0x88, 0x88,

    /* U+0073 "s" */
    0x1b, 0x70, 0xe1, 0xc5, 0xc0,

    /* U+0074 "t" */
    0x21, 0x3e, 0x42, 0x10, 0x84,

    /* U+0075 "u" */
    0x8c, 0x63, 0x18, 0xbc,

    /* U+0076 "v" */
    0x8c, 0x54, 0xa6, 0x10,

    /* U+0077 "w" */
    0xc9, 0x5b, 0x5a, 0x6a, 0x66, 0x64,

    /* U+0078 "x" */
    0x46, 0x58, 0x61, 0xc6, 0xd0, 0x80,

    /* U+0079 "y" */
    0x87, 0x24, 0x9c, 0x30, 0x82, 0x18, 0x40,

    /* U+007A "z" */
    0xf8, 0x8c, 0xcc, 0x7c
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 98, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 5, .adv_w = 114, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 12, .adv_w = 99, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 16, .adv_w = 113, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 23, .adv_w = 105, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 28, .adv_w = 98, .box_w = 6, .box_h = 10, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 36, .adv_w = 102, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 43, .adv_w = 111, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 49, .adv_w = 54, .box_w = 2, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 52, .adv_w = 77, .box_w = 4, .box_h = 12, .ofs_x = -1, .ofs_y = -3},
    {.bitmap_index = 58, .adv_w = 104, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 64, .adv_w = 53, .box_w = 1, .box_h = 9, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 66, .adv_w = 149, .box_w = 8, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 72, .adv_w = 101, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 76, .adv_w = 101, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 80, .adv_w = 103, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 86, .adv_w = 100, .box_w = 5, .box_h = 9, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 92, .adv_w = 92, .box_w = 4, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 95, .adv_w = 93, .box_w = 5, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 100, .adv_w = 90, .box_w = 5, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 105, .adv_w = 100, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 109, .adv_w = 93, .box_w = 5, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 113, .adv_w = 131, .box_w = 8, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 119, .adv_w = 113, .box_w = 7, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 125, .adv_w = 100, .box_w = 6, .box_h = 9, .ofs_x = 0, .ofs_y = -3},
    {.bitmap_index = 132, .adv_w = 103, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 97, .range_length = 26, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif

};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t comic_sans = {
#else
lv_font_t comic_sans = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .release_glyph = NULL,                             /*Static font: nothing to release*/
    .line_height = 12,          /*The maximum line height required by the font*/
    .base_line = 3,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -2,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if COMIC_SANS*/
