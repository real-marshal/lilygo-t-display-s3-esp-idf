#include "UI.hpp"
#include "lvgl.h"

namespace {

void ta_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* ta = lv_event_get_target(e);
  lv_obj_t* kb = (lv_obj_t*)lv_event_get_user_data(e);
  if (code == LV_EVENT_FOCUSED) {
    lv_keyboard_set_textarea(kb, ta);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }

  if (code == LV_EVENT_DEFOCUSED) {
    lv_keyboard_set_textarea(kb, NULL);
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }
}

struct KBUserData {
  lv_obj_t* kb;
  lv_obj_t* ta;
};

void kb_event_cb(lv_event_t* e) {
  KBUserData* kbUserData = (KBUserData*)lv_event_get_user_data(e);
  lv_obj_add_flag(kbUserData->kb, LV_OBJ_FLAG_HIDDEN);
  lv_indev_reset(NULL, kbUserData->ta);
}

void lv_example_keyboard_1(void) {
  /*Create a text area. The keyboard will write here*/
  lv_obj_t* ta;
  ta = lv_textarea_create(lv_scr_act());
  lv_obj_align(ta, LV_ALIGN_TOP_LEFT, 5, 5);
  lv_textarea_set_placeholder_text(ta, "Hmmm...");
  lv_obj_set_size(ta, 310, 75);

  /*Create a keyboard to use it with an of the text areas*/
  lv_obj_t* kb = lv_keyboard_create(lv_scr_act());
  lv_keyboard_set_popovers(kb, true);

  lv_obj_add_event_cb(ta, ta_event_cb, LV_EVENT_ALL, kb);

  static KBUserData kbUserData(kb, ta);

  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_READY, &kbUserData);
  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_CANCEL, &kbUserData);

  lv_keyboard_set_textarea(kb, ta);
}

}  // namespace

void renderUI(lv_disp_t* lvDisp) {
  lv_example_keyboard_1();
}