/* nesp32's miniz_export.h (see miniz.h). miniz's names are given a prefix
 * of nesp32's: a program that links this code beside a miniz of its own
 * (an emulator) or the ESP32's ROM's has two of none of them. */
#pragma once

#define MINIZ_EXPORT
#define tinfl_decompress nesp_tinfl_decompress
#define tinfl_decompress_mem_to_heap nesp_tinfl_decompress_mem_to_heap
#define tinfl_decompress_mem_to_mem nesp_tinfl_decompress_mem_to_mem
#define tinfl_decompress_mem_to_callback nesp_tinfl_decompress_mem_to_callback
#define tinfl_decompressor_alloc nesp_tinfl_decompressor_alloc
#define tinfl_decompressor_free nesp_tinfl_decompressor_free
#define miniz_def_alloc_func nesp_miniz_def_alloc_func
#define miniz_def_free_func nesp_miniz_def_free_func
#define miniz_def_realloc_func nesp_miniz_def_realloc_func
