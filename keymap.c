#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#define MOON_LED_LEVEL LED_LEVEL

enum custom_keycodes {
    RGB_SLD = ML_SAFE_RANGE,
    ST_MACRO_0,
    ST_MACRO_1,
    ST_MACRO_2,
};

enum tap_dance_codes {
    DANCE_0,
    DANCE_CTRL_ALT,
    DANCE_LANG_S,
    DANCE_LANG_I,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_EQUAL,              KC_1,               KC_2,             KC_3,            KC_4,           KC_5,                                                           KC_6,                   KC_7,           KC_8,             KC_9,           KC_0,                   KC_MINUS,
    KC_ESCAPE,             KC_Q,               KC_W,             KC_E,            KC_R,           KC_T,                                                           KC_Y,                   KC_U,           KC_I,             KC_O,           KC_P,                   KC_BSLS,
    MT(MOD_LSFT, KC_BSPC), KC_A,               KC_S,             KC_D,            KC_F,           KC_G,                                                           KC_H,                   KC_J,           KC_K,             KC_L,           KC_SCLN,                MT(MOD_RSFT, KC_QUOTE),
    TD(DANCE_CTRL_ALT),    MT(MOD_LALT, KC_Z), KC_X,             KC_C,            KC_V,           KC_B,                                                           KC_N,                   KC_M,           KC_COMMA,         KC_DOT,         MT(MOD_RALT, KC_SLASH), KC_RIGHT_CTRL,
                                                                                                  LT(1,KC_SPACE),  MT(MOD_LGUI, KC_TAB),  MT(MOD_RCTL, KC_ENTER), LT(2,KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_TILD,               KC_F1,              KC_F2,            KC_F3,           KC_F4,          KC_F5,                                                          KC_F6,                  KC_F7,          KC_F8,            KC_F9,          KC_F10,                 KC_F11,
    KC_GRAVE,              KC_EXLM,            KC_AT,            KC_HASH,         KC_DLR,         KC_PERC,                                                        KC_7,                   KC_8,           KC_9,             KC_MINUS,       KC_SLASH,               KC_F12,
    KC_TRANSPARENT,        KC_CIRC,            KC_AMPR,          KC_ASTR,         KC_LPRN,        KC_RPRN,                                                        KC_4,                   KC_5,           KC_6,             KC_PLUS,        KC_ASTR,                KC_BSPC,
    KC_TRANSPARENT,        KC_TRANSPARENT,     KC_LBRC,          KC_RBRC,         KC_LCBR,        KC_RCBR,                                                        KC_1,                   KC_2,           KC_3,             KC_DOT,         KC_EQUAL,               KC_ENTER,
                                                                                                  KC_TRANSPARENT, KC_TRANSPARENT,                 KC_TRANSPARENT, KC_0
  ),
  [2] = LAYOUT_voyager(
    RGB_TOG,               TOGGLE_LAYER_COLOR, RGB_MODE_FORWARD, RGB_SLD,         RGB_VAD,        RGB_VAI,                                                        KC_TRANSPARENT,         KC_TRANSPARENT, KC_TRANSPARENT,   KC_TRANSPARENT, KC_TRANSPARENT,         QK_BOOT,
    KC_TRANSPARENT,        LGUI(KC_Q),         TD(DANCE_0),      KC_AUDIO_VOL_UP, KC_TRANSPARENT, KC_TRANSPARENT,                                                 KC_TRANSPARENT,         KC_PC_UNDO,     KC_TRANSPARENT,   LGUI(KC_GRAVE), KC_MAC_PASTE,           KC_TRANSPARENT,
    KC_TRANSPARENT,        KC_TRANSPARENT,     KC_TRANSPARENT,   KC_TRANSPARENT,  KC_TRANSPARENT, KC_TRANSPARENT,                                                 KC_LEFT,                KC_DOWN,        KC_UP,            KC_RIGHT,       KC_TRANSPARENT,         KC_TRANSPARENT,
    KC_TRANSPARENT,        KC_TRANSPARENT,     KC_TRANSPARENT,   KC_MAC_COPY,     KC_TRANSPARENT, KC_TRANSPARENT,                                                 LGUI(KC_GRAVE),         KC_TRANSPARENT, ST_MACRO_1,       ST_MACRO_2,     KC_TRANSPARENT,         KC_TRANSPARENT,
                                                                                                  KC_TRANSPARENT, KC_TRANSPARENT,                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT,        KC_TRANSPARENT,     KC_TRANSPARENT,   KC_TRANSPARENT,  KC_TRANSPARENT, KC_TRANSPARENT,                                                 KC_TRANSPARENT,         KC_TRANSPARENT, KC_TRANSPARENT,   KC_TRANSPARENT, KC_TRANSPARENT,         KC_TRANSPARENT,
    KC_TRANSPARENT,        KC_TRANSPARENT,     KC_D,             KC_T,            KC_H,           KC_N,                                                           KC_M,                   KC_E,           TD(DANCE_LANG_I), KC_J,           KC_G,                   KC_TRANSPARENT,
    KC_TRANSPARENT,        KC_F,               TD(DANCE_LANG_S), KC_L,            KC_A,           KC_U,                                                           KC_LEFT_BRACKET,        KC_SEMICOLON,   KC_R,             KC_K,           KC_TRANSPARENT,         KC_TRANSPARENT,
    KC_TRANSPARENT,        MT(MOD_LALT, KC_P), KC_LEFT_BRACKET,  KC_X,            KC_D,           KC_COMMA,                                                       KC_Y,                   KC_V,           KC_TRANSPARENT,   KC_TRANSPARENT, KC_TRANSPARENT,         KC_TRANSPARENT,
                                                                                                  KC_TRANSPARENT, KC_TRANSPARENT,                     KC_MS_BTN1, KC_TRANSPARENT
  ),
};

enum combos {
    LANG_SWITCH,
    LANG_YU,
    LANG_YA,
};

const uint16_t PROGMEM combo_lang_change[] = { LT(2, KC_SPACE), LT(1, KC_SPACE), COMBO_END };
const uint16_t PROGMEM combo_lang_yu[]     = { KC_M, KC_E, COMBO_END };
const uint16_t PROGMEM combo_lang_ya[]     = { KC_M, KC_F, COMBO_END };

combo_t key_combos[COMBO_COUNT] = {
    [LANG_SWITCH] = COMBO(combo_lang_change, ST_MACRO_0),
    [LANG_YU] = COMBO(combo_lang_yu, KC_DOT),
    [LANG_YA] = COMBO(combo_lang_ya, KC_Z),
};

bool combo_should_trigger(uint16_t combo_idx, combo_t *combo, uint16_t keycode, keyrecord_t *record) {
    switch (combo_idx) {
        case LANG_YU:
        case LANG_YA:
            return layer_state_is(3);
    }

    return true;
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {139,216,206}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {202,219,201}, {139,216,206}, {202,219,201}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {202,219,201}, {202,219,201}, {202,219,201}, {139,216,206}, {82,219,201}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,8,230}, {82,219,201}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {202,219,201}, {202,219,201}, {202,219,201}, {202,219,201}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {139,216,206}, {82,219,201}, {0,255,255}, {139,8,230} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {202,219,201}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {179,219,201}, {0,0,0}, {53,219,201}, {202,219,201}, {0,0,0}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {179,219,201}, {179,219,201}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 2:
      set_layer_color(2);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      layer_invert(3);
      SEND_STRING(SS_LCTL(SS_TAP(X_SPACE)));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_LGUI(SS_TAP(X_LEFT))));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_LGUI(SS_TAP(X_RIGHT))));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}

// ----------------------------------------
// Tap Dance
// ----------------------------------------

typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[1];

uint8_t dance_step(tap_dance_state_t *state);

void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void dance_ctrl_alt_finished(tap_dance_state_t *state, void *user_data);
void dance_ctrl_alt_reset(tap_dance_state_t *state, void *user_data);

tap_dance_action_t tap_dance_actions[] = {
    [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
    [DANCE_CTRL_ALT] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_ctrl_alt_finished, dance_ctrl_alt_reset),
    [DANCE_LANG_S] = ACTION_TAP_DANCE_DOUBLE(KC_C, KC_I),
    [DANCE_LANG_I] = ACTION_TAP_DANCE_DOUBLE(KC_B, KC_S),
};

// ----------------------------------------
// DANCE_0: Long press W to close a tab (macOs)
// ----------------------------------------

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_AUDIO_VOL_DOWN);
        tap_code16(KC_AUDIO_VOL_DOWN);
        tap_code16(KC_AUDIO_VOL_DOWN);
    }
    if(state->count > 3) {
        tap_code16(KC_AUDIO_VOL_DOWN);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_AUDIO_VOL_DOWN); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_W)); break;
        case DOUBLE_TAP: register_code16(KC_AUDIO_VOL_DOWN); register_code16(KC_AUDIO_VOL_DOWN); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_AUDIO_VOL_DOWN); register_code16(KC_AUDIO_VOL_DOWN);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_AUDIO_VOL_DOWN); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_W)); break;
        case DOUBLE_TAP: unregister_code16(KC_AUDIO_VOL_DOWN); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_AUDIO_VOL_DOWN); break;
    }
    dance_state[0].step = 0;
}

// ----------------------------------------
// DANCE_CTRL_ALT: Double press and hold CTRL to get ALT single holded
// ----------------------------------------

void dance_ctrl_alt_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code(KC_LEFT_CTRL); break;
        case SINGLE_HOLD: register_code(KC_LEFT_CTRL); break;
        case DOUBLE_HOLD: register_code(KC_LEFT_ALT); break;
    }
}

void dance_ctrl_alt_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code(KC_LEFT_CTRL); break;
        case SINGLE_HOLD: unregister_code(KC_LEFT_CTRL); break;
        case DOUBLE_HOLD: unregister_code(KC_LEFT_ALT); break;
    }
    dance_state[0].step = 0;
}
