#ifndef UI_TYPES_H
#define UI_TYPES_H

#include "referee_protocol.h"

typedef Graph_Data_t ui_interface_figure_t;
typedef Graph_Data_t ui_interface_line_t;
typedef Graph_Data_t ui_interface_rect_t;
typedef Graph_Data_t ui_interface_round_t;
typedef Graph_Data_t ui_interface_ellipse_t;
typedef Graph_Data_t ui_interface_arc_t;
typedef Graph_Data_t ui_interface_number_t;
typedef String_Data_t ui_interface_string_t;

#pragma pack(1)
typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    ui_interface_figure_t data[1];
    uint16_t frametail;
} ui_1_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    ui_interface_figure_t data[2];
    uint16_t frametail;
} ui_2_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    ui_interface_figure_t data[5];
    uint16_t frametail;
} ui_5_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    ui_interface_figure_t data[7];
    uint16_t frametail;
} ui_7_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    ui_interface_string_t option;
    uint16_t frametail;
} ui_string_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t frametail;
} ui_delete_frame_t;
#pragma pack()

#endif
