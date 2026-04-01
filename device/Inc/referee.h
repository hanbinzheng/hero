#ifndef REFEREE_H_
#define REFEREE_H_

/* revised from Yaofang Ji https://github.com/Froze23n/ */
#include <stdint.h>
#include "usart.h"

/* enum and struct definition */

enum referee_cmd_id {
        ID_GAME_STATUS = 0x0001,   /* Game status, 1Hz */
        ID_GAME_RESULT = 0x0002,   /* Game result triggered */
        ID_GAME_ROBOT_HP = 0x0003, /* Robot HP, 3Hz */

        ID_EVENT_DATA = 0x0101, /* Field event, 1Hz */
        /* Referee warning data triggered, 1Hz */
        ID_REFEREE_WARNING = 0x104,
        /* Dart launch data 1Hz */
        ID_DART_INFO = 0x105,

        /* Robot performance system data, 10Hz */
        ID_ROBOT_STATUS = 0x0201,
        /* Chassis buffer energy & shooting heat, 10Hz */
        ID_POWER_HEAT_DATA = 0x0202,
        ID_ROBOT_POS = 0x0203,           /* Robot position, 1Hz */
        ID_BUFF = 0x0204,                /* Robot buff & chassis energy, 3Hz */
        ID_HURT_DATA = 0x0206,           /* Hurt status triggered */
        ID_SHOOT_DATA = 0x0207,          /* Real-time shooting triggered */
        ID_PROJECTILE_ALLOWANCE = 0x208, /* Ammo allowance, 10Hz */
        ID_RFID_STATUS = 0x209,          /* Robot RFID module status, 3Hz */
        /* 0x20A - 0x20E (Reserved for dart/sentry/radar) */

        /* Inter-robot interaction, triggered by sender, max 30Hz */
        ID_ROBOT_INTERACTION_DATA = 0x0301,
        /* 0x302 (video link) */
        /* 0x303 Competitor minimap interaction data competitor->robot */
        /* 0x305 (Radar related) */
        /* 0x306 (Custom controller) */
        /* 0x307 (Sentry related) */
        /* 0x308 Competitor minimap interaction data robot->competitor */
        /* 0x309-0x311 (Video link) */
        /* 0xA01-0xA06 (Radar wireless link) */
};

#pragma pack(push, 1)
struct game_status {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t sync_time_stamp;
}; /* 0x0001 */

struct game_result {
        uint8_t winner;
}; /* 0x0002 */

struct game_robot_hp {
        uint16_t ally_1_robot_hp;
        uint16_t ally_2_robot_hp;
        uint16_t ally_3_robot_hp;
        uint16_t ally_4_robot_hp;
        uint16_t reserved;
        uint16_t ally_7_robot_hp;
        uint16_t ally_outpost_hp;
        uint16_t ally_base_hp;
}; /* 0x0003 */

struct event_data {
        uint32_t event_data;
}; /* 0x0101 */

struct referee_warning {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
}; /* 0x0104 */

struct dart_info {
        uint8_t dart_remaining_time;
        uint16_t dart_info;
}; /* 0x0105 */

struct robot_status {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_hp;
        uint16_t maximum_hp;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
}; /* 0x0201 */

struct power_heat_data {
        /* uint16_t x 2 + float: 8 byte in total */
        uint8_t reserved[8];
        uint16_t buffer_energy;
        uint16_t shooter_17mm_barrel_heat;
        uint16_t shooter_42mm_barrel_heat;
}; /* 0x0202 */

struct robot_pos {
        float x;
        float y;
        float angle;
}; /* 0x0203 */

struct buff {
        uint8_t recovery_buff;
        uint16_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
}; /* 0x0204 */

struct hurt_data {
        uint8_t armor_id : 4;
        uint8_t hp_deduction_reason : 4;
}; /* 0x0206 */

struct shoot_data {
        uint8_t bullet_type;
        uint8_t shooter_number;
        uint8_t launching_frequency;
        float initial_speed;
}; /* 0x0207 */

struct projectile_allowance {
        uint16_t projectile_allowance_17mm;
        uint16_t projectile_allowance_42mm;
        uint16_t remaining_gold_coin;
        uint16_t projectile_allowance_fortress;
}; /* 0x0208 */

struct rfid_status {
        uint32_t rfid_status;
        uint8_t rfid_status_2;
}; /* 0x0209 */

struct interaction_figure {
        uint8_t figure_name[3]; /* byte 0 - 2 */

        /* byte 3 - 6: figure configuration 1, a uint32_t  */

        /* bit 0 - 2: operation of figure
         * 0: None, 1: add, 2: modify, 3: delete
         */
        uint32_t operate_type : 3;

        /* bit 3 - 5: type of figure
         * 0: line, 1: rectangle, 2: circle, 3: ellipse
         * 4: cyclometer, 5: float number, 6: integer, 7: characters
         */
        uint32_t figure_type : 3;

        /* bit 6 - 9: number of layers, 0 - 9 */
        uint32_t layer : 4;

        /* bit 10 - 13: color
         * 0: red/blue (color of our side), 1: yellow, 2: green, 
         * 3: orange, 4: purple, 5: pink, 6: cyan, 7: black, 8: white
         */
        uint32_t color : 4;

        /* bit 14 - 31: figure details, different according to figure type */
        uint32_t details_a : 9;
        uint32_t details_b : 9;

        /* byte 7 - 10: figure configuration 2, a uint32_t */
        uint32_t width : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;

        /* byte 11 - 14: figure configuration 3, a uint32_t
         *different according to figure type
         */
        uint32_t details_c : 10;
        uint32_t details_d : 11;
        uint32_t details_e : 11;
};
#pragma pack(pop)

struct referee_info {
        struct game_status game_status;
        struct game_result game_result;
        struct game_robot_hp game_robot_hp;

        struct event_data event_data;
        struct referee_warning referee_warning;
        struct dart_info dart_info;

        struct robot_status robot_status;
        struct power_heat_data power_heat_data;
        struct robot_pos robot_pos;
        struct buff buff;
        struct hurt_data hurt_data;
        struct shoot_data shoot_data;
        struct projectile_allowance projectile_allowance;
        struct rfid_status rfid_status;
};

/* global variable and functino declaration */
extern struct referee_info referee_info;

HAL_StatusTypeDef referee_ui_update(void);

#endif /* REFEREE_H_ */
