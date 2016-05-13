#ifdef __AVR__
  #include <avr/pgmspace.h>
  static const char PROGMEM  font5x8[] = {
#else 
  static const char  font5x8[] = {
#endif
		  0x00, 0x00, 0x00, 0x00,0x00,// (spacja)
		  0x00, 0x00, 0x5F, 0x00,0x00,// !
		  0x00, 0x07, 0x00, 0x07,0x00,// """
		  0x14, 0x7F, 0x14, 0x7F,0x14,// #
		  0x12, 0x2A, 0x7F, 0x2A,0x24,// $
		  0x62, 0x64, 0x08, 0x13,0x23,// %
		  0x50, 0x22, 0x55, 0x49,0x36,// &
		  0x00, 0x00, 0x03, 0x05,0x00,// '
		  0x00, 0x41, 0x22, 0x1C,0x00,// (
		  0x00, 0x1C, 0x22, 0x41,0x00,// )
		  0x08, 0x2A, 0x1C, 0x2A,0x08,// *
		  0x08, 0x08, 0x3E, 0x08,0x08,// +
		  0x00, 0x00, 0x30, 0x50,0x00,//
		  0x08, 0x08, 0x08, 0x08,0x08,// -
		  0x00, 0x00, 0x30, 0x30,0x00,// .
		  0x02, 0x04, 0x08, 0x10,0x20,// /
		  0x3E, 0x45, 0x49, 0x51,0x3E,// 0
		  0x00, 0x40, 0x7F, 0x42,0x00,// 1
		  0x46, 0x49, 0x51, 0x61,0x42,// 2
		  0x31, 0x4B, 0x45, 0x41,0x21,// 3
		  0x10, 0x7F, 0x12, 0x14,0x18,// 4
		  0x39, 0x45, 0x45, 0x45,0x27,// 5
		  0x30, 0x49, 0x49, 0x4A,0x3C,// 6
		  0x03, 0x05, 0x09, 0x71,0x01,// 7
		  0x36, 0x49, 0x49, 0x49,0x36,// 8
		  0x1E, 0x29, 0x49, 0x49,0x06,// 9
		  0x00, 0x00, 0x36, 0x36,0x00,// :
		  0x00, 0x00, 0x36, 0x56,0x00,// ,"
		  0x41, 0x22, 0x14, 0x08,0x00,// <
		  0x14, 0x14, 0x14, 0x14,0x14,// =
		  0x00, 0x08, 0x14, 0x22,0x41,// >
		  0x06, 0x09, 0x51, 0x01,0x02,// ?
		  0x3E, 0x41, 0x79, 0x49,0x32,// @
		  0x7E, 0x11, 0x11, 0x11,0x7E,// A
		  0x36, 0x49, 0x49, 0x49,0x7F,// B
		  0x22, 0x41, 0x41, 0x41,0x3E,// C
		  0x1C, 0x22, 0x41, 0x41,0x7F,// D
		  0x41, 0x49, 0x49, 0x49,0x7F,// E
		  0x01, 0x01, 0x09, 0x09,0x7F,// F
		  0x32, 0x51, 0x41, 0x41,0x3E,// G
		  0x7F, 0x08, 0x08, 0x08,0x7F,// H
		  0x00, 0x41, 0x7F, 0x41,0x00,// I
		  0x01, 0x3F, 0x41, 0x40,0x20,// J
		  0x41, 0x22, 0x14, 0x08,0x7F,// K
		  0x40, 0x40, 0x40, 0x40,0x7F,// L
		  0x7F, 0x02, 0x04, 0x02,0x7F,// M
		  0x7F, 0x10, 0x08, 0x04,0x7F,// N
		  0x3E, 0x41, 0x41, 0x41,0x3E,// O
		  0x06, 0x09, 0x09, 0x09,0x7F,// P
		  0x5E, 0x21, 0x51, 0x41,0x3E,// Q
		  0x46, 0x29, 0x19, 0x09,0x7F,// R
		  0x31, 0x49, 0x49, 0x49,0x46,// S
		  0x01, 0x01, 0x7F, 0x01,0x01,// T
		  0x3F, 0x40, 0x40, 0x40,0x3F,// U
		  0x1F, 0x20, 0x40, 0x20,0x1F,// V
		  0x7F, 0x20, 0x18, 0x20,0x7F,// W
		  0x63, 0x14, 0x08, 0x14,0x63,// X
		  0x03, 0x04, 0x78, 0x04,0x03,// Y
		  0x43, 0x45, 0x49, 0x51,0x61,// Z
		  0x41, 0x41, 0x7F, 0x00,0x00,// [
		  0x20, 0x10, 0x08, 0x04,0x02,// ""\"""
		  0x00, 0x00, 0x7F, 0x41,0x41,// ]
		  0x04, 0x02, 0x01, 0x02,0x04,// ^
		  0x40, 0x40, 0x40, 0x40,0x40,// _
		  0x00, 0x04, 0x02, 0x01,0x00,// `
		  0x78, 0x54, 0x54, 0x54,0x20,// a
		  0x38, 0x44, 0x44, 0x48,0x7F,// b
		  0x20, 0x44, 0x44, 0x44,0x38,// c
		  0x7F, 0x48, 0x44, 0x44,0x38,// d
		  0x18, 0x54, 0x54, 0x54,0x38,// e
		  0x02, 0x01, 0x09, 0x7E,0x08,// f
		  0x3C, 0x54, 0x54, 0x14,0x08,// g
		  0x78, 0x04, 0x04, 0x08,0x7F,// h
		  0x00, 0x40, 0x7D, 0x44,0x00,// i
		  0x00, 0x3D, 0x44, 0x40,0x20,// j
		  0x44, 0x28, 0x10, 0x7F,0x00,// k
		  0x00, 0x40, 0x7F, 0x41,0x00,// l
		  0x78, 0x04, 0x18, 0x04,0x7C,// m
		  0x78, 0x04, 0x04, 0x08,0x7C,// n
		  0x38, 0x44, 0x44, 0x44,0x38,// o
		  0x08, 0x14, 0x14, 0x14,0x7C,// p
		  0x7C, 0x18, 0x14, 0x14,0x08,// q
		  0x08, 0x04, 0x04, 0x08,0x7C,// r
		  0x20, 0x54, 0x54, 0x54,0x48,// s
		  0x20, 0x40, 0x44, 0x3F,0x04,// t
		  0x7C, 0x20, 0x40, 0x40,0x3C,// u
		  0x1C, 0x20, 0x40, 0x20,0x1C,// v
		  0x3C, 0x40, 0x30, 0x40,0x3C,// w
		  0x44, 0x28, 0x10, 0x28,0x44,// x
		  0x3C, 0x50, 0x50, 0x50,0x0C,// y
		  0x44, 0x4C, 0x54, 0x64,0x44,// z
		  0x00, 0x41, 0x36, 0x08,0x00,// {
		  0x00, 0x00, 0x7F, 0x00,0x00,// |
		  0x00, 0x08, 0x36, 0x41,0x00,// }
		  0x08, 0x1C, 0x2A, 0x08,0x08,// ->
		  0x08, 0x08, 0x2A, 0x1C,0x08 // <-


//0x00, 0x00, 0x00, 0x00, 0x00,// (spacja)
//0x00, 0x00, 0x5F, 0x00, 0x00,// !
//0x00, 0x07, 0x00, 0x07, 0x00,// "
//0x14, 0x7F, 0x14, 0x7F, 0x14,// #
//0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
//0x23, 0x13, 0x08, 0x64, 0x62,// %
//0x36, 0x49, 0x55, 0x22, 0x50,// &
//0x00, 0x05, 0x03, 0x00, 0x00,// '
//0x00, 0x1C, 0x22, 0x41, 0x00,// (
//0x00, 0x41, 0x22, 0x1C, 0x00,// )
//0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
//0x08, 0x08, 0x3E, 0x08, 0x08,// +
//0x00, 0x50, 0x30, 0x00, 0x00,// ,
//0x08, 0x08, 0x08, 0x08, 0x08,// -
//0x00, 0x30, 0x30, 0x00, 0x00,// .
//0x20, 0x10, 0x08, 0x04, 0x02,// /
//0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
//0x00, 0x42, 0x7F, 0x40, 0x00,// 1
//0x42, 0x61, 0x51, 0x49, 0x46,// 2
//0x21, 0x41, 0x45, 0x4B, 0x31,// 3
//0x18, 0x14, 0x12, 0x7F, 0x10,// 4
//0x27, 0x45, 0x45, 0x45, 0x39,// 5
//0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
//0x01, 0x71, 0x09, 0x05, 0x03,// 7
//0x36, 0x49, 0x49, 0x49, 0x36,// 8
//0x06, 0x49, 0x49, 0x29, 0x1E,// 9
//0x00, 0x36, 0x36, 0x00, 0x00,// :
//0x00, 0x56, 0x36, 0x00, 0x00,// ;
//0x00, 0x08, 0x14, 0x22, 0x41,// <
//0x14, 0x14, 0x14, 0x14, 0x14,// =
//0x41, 0x22, 0x14, 0x08, 0x00,// >
//0x02, 0x01, 0x51, 0x09, 0x06,// ?
//0x32, 0x49, 0x79, 0x41, 0x3E,// @
//0x7E, 0x11, 0x11, 0x11, 0x7E,// A
//0x7F, 0x49, 0x49, 0x49, 0x36,// B
//0x3E, 0x41, 0x41, 0x41, 0x22,// C
//0x7F, 0x41, 0x41, 0x22, 0x1C,// D
//0x7F, 0x49, 0x49, 0x49, 0x41,// E
//0x7F, 0x09, 0x09, 0x01, 0x01,// F
//0x3E, 0x41, 0x41, 0x51, 0x32,// G
//0x7F, 0x08, 0x08, 0x08, 0x7F,// H
//0x00, 0x41, 0x7F, 0x41, 0x00,// I
//0x20, 0x40, 0x41, 0x3F, 0x01,// J
//0x7F, 0x08, 0x14, 0x22, 0x41,// K
//0x7F, 0x40, 0x40, 0x40, 0x40,// L
//0x7F, 0x02, 0x04, 0x02, 0x7F,// M
//0x7F, 0x04, 0x08, 0x10, 0x7F,// N
//0x3E, 0x41, 0x41, 0x41, 0x3E,// O
//0x7F, 0x09, 0x09, 0x09, 0x06,// P
//0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
//0x7F, 0x09, 0x19, 0x29, 0x46,// R
//0x46, 0x49, 0x49, 0x49, 0x31,// S
//0x01, 0x01, 0x7F, 0x01, 0x01,// T
//0x3F, 0x40, 0x40, 0x40, 0x3F,// U
//0x1F, 0x20, 0x40, 0x20, 0x1F,// V
//0x7F, 0x20, 0x18, 0x20, 0x7F,// W
//0x63, 0x14, 0x08, 0x14, 0x63,// X
//0x03, 0x04, 0x78, 0x04, 0x03,// Y
//0x61, 0x51, 0x49, 0x45, 0x43,// Z
//0x00, 0x00, 0x7F, 0x41, 0x41,// [
//0x02, 0x04, 0x08, 0x10, 0x20,// "\"
//0x41, 0x41, 0x7F, 0x00, 0x00,// ]
//0x04, 0x02, 0x01, 0x02, 0x04,// ^
//0x40, 0x40, 0x40, 0x40, 0x40,// _
//0x00, 0x01, 0x02, 0x04, 0x00,// `
//0x20, 0x54, 0x54, 0x54, 0x78,// a
//0x7F, 0x48, 0x44, 0x44, 0x38,// b
//0x38, 0x44, 0x44, 0x44, 0x20,// c
//0x38, 0x44, 0x44, 0x48, 0x7F,// d
//0x38, 0x54, 0x54, 0x54, 0x18,// e
//0x08, 0x7E, 0x09, 0x01, 0x02,// f
//0x08, 0x14, 0x54, 0x54, 0x3C,// g
//0x7F, 0x08, 0x04, 0x04, 0x78,// h
//0x00, 0x44, 0x7D, 0x40, 0x00,// i
//0x20, 0x40, 0x44, 0x3D, 0x00,// j
//0x00, 0x7F, 0x10, 0x28, 0x44,// k
//0x00, 0x41, 0x7F, 0x40, 0x00,// l
//0x7C, 0x04, 0x18, 0x04, 0x78,// m
//0x7C, 0x08, 0x04, 0x04, 0x78,// n
//0x38, 0x44, 0x44, 0x44, 0x38,// o
//0x7C, 0x14, 0x14, 0x14, 0x08,// p
//0x08, 0x14, 0x14, 0x18, 0x7C,// q
//0x7C, 0x08, 0x04, 0x04, 0x08,// r
//0x48, 0x54, 0x54, 0x54, 0x20,// s
//0x04, 0x3F, 0x44, 0x40, 0x20,// t
//0x3C, 0x40, 0x40, 0x20, 0x7C,// u
//0x1C, 0x20, 0x40, 0x20, 0x1C,// v
//0x3C, 0x40, 0x30, 0x40, 0x3C,// w
//0x44, 0x28, 0x10, 0x28, 0x44,// x
//0x0C, 0x50, 0x50, 0x50, 0x3C,// y
//0x44, 0x64, 0x54, 0x4C, 0x44,// z
//0x00, 0x08, 0x36, 0x41, 0x00,// {
//0x00, 0x00, 0x7F, 0x00, 0x00,// |
//0x00, 0x41, 0x36, 0x08, 0x00,// }
//0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
//0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};


  static const unsigned char reverse_table[] = {
            0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
            0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
            0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
            0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
            0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
            0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
            0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
            0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
            0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
            0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
            0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
            0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
            0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
            0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
            0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
            0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
            0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
            0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
            0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
            0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
            0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
            0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
            0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
            0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
            0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
            0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
            0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
            0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
            0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
            0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
            0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
            0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
        };
  unsigned char reverse_byte(unsigned char x)
    {
  	  return reverse_table[x];
  	}
//
