
/** @file
 * Unit tests of the parameter configuration infrastructure.
 */

#include <limits>

#include <gtest/gtest.h>

#include "../param.h"

using namespace std;

typedef enum MAV_PARAM_TYPE mav_param_type_t;

template <typename T> enum MAV_PARAM_TYPE
mav_param_type();

template <> enum MAV_PARAM_TYPE
mav_param_type<int8_t>() {return MAV_PARAM_TYPE_INT8;}

template <> enum MAV_PARAM_TYPE
mav_param_type<int16_t>() {return MAV_PARAM_TYPE_INT16;}

template <> enum MAV_PARAM_TYPE
mav_param_type<int32_t>() {return MAV_PARAM_TYPE_INT32;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint8_t>() {return MAV_PARAM_TYPE_UINT8;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint16_t>() {return MAV_PARAM_TYPE_UINT16;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint32_t>() {return MAV_PARAM_TYPE_UINT32;}

template <> enum MAV_PARAM_TYPE
mav_param_type<float>() {return MAV_PARAM_TYPE_REAL32;}


/// Get value of param_value_union_t
template <typename T> T
punion_value(param_value_union_t);

/// Get int8_t value of param_value_union_t
template <> int8_t
punion_value(param_value_union_t punion) {
  return punion.param_int8;
}

/// Get int16_t value of param_value_union_t
template <> int16_t
punion_value(param_value_union_t punion) {
  return punion.param_int16;
}

/// Get int32_t value of param_value_union_t
template <> int32_t
punion_value(param_value_union_t punion) {
  return punion.param_int32;
}

/// Get uint8_t value of param_value_union_t
template <> uint8_t
punion_value(param_value_union_t punion) {
  return punion.param_uint8;
}

/// Get uint16_t value of param_value_union_t
template <> uint16_t
punion_value(param_value_union_t punion) {
  return punion.param_uint16;
}

/// Get uint32_t value of param_value_union_t
template <> uint32_t
punion_value(param_value_union_t punion) {
  return punion.param_uint32;
}

/// Get float value of param_value_union_t
template <> float
punion_value(param_value_union_t punion) {
  return punion.param_float;
}

/// Set value of param_value_union_t
template <typename T> param_value_union_t 
make_punion(T);

/// Set int8_t value of param_value_union_t
template <> param_value_union_t
make_punion(int8_t value) {
  param_value_union_t punion;
  punion.param_int8 = value;
  return punion;
}

/// Set int16_t value of param_value_union_t
template <> param_value_union_t
make_punion(int16_t value) {
  param_value_union_t punion;
  punion.param_int16 = value;
  return punion;
}

/// Set int32_t value of param_value_union_t
template <> param_value_union_t
make_punion(int32_t value) {
  param_value_union_t punion;
  punion.param_int32 = value;
  return punion;
}

/// Set uint8_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint8_t value) {
  param_value_union_t punion;
  punion.param_uint8 = value;
  return punion;
}

/// Set uint16_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint16_t value) {
  param_value_union_t punion;
  punion.param_uint16 = value;
  return punion;
}

/// Set uint32_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint32_t value) {
  param_value_union_t punion;
  punion.param_uint32 = value;
  return punion;
}

/// Set float value of param_value_union_t
template <> param_value_union_t
make_punion(float value) {
  param_value_union_t punion;
  punion.param_float = value;
  return punion;
}

/// Parameter test fixture
template <typename T>
class ParamTest : public ::testing::Test {
  
};

typedef ::testing::Types<int8_t, int16_t, int32_t, 
                         uint8_t, uint16_t, uint32_t, float> ParamTypes;
TYPED_TEST_CASE(ParamTest, ParamTypes);


/// Test param get/set functions.
TYPED_TEST(ParamTest, GetSet) {
  TypeParam min = numeric_limits<TypeParam>::min();
  TypeParam max = numeric_limits<TypeParam>::max();
  
  TypeParam val = min;
  param_t param = {};
  param.type = mav_param_type<TypeParam>();
  param.data = &val;
  EXPECT_EQ(punion_value<TypeParam>(param_get(&param)), min);
  
  param_value_union_t new_value = make_punion(max);
  EXPECT_EQ(param_set(&param, new_value), STATUS_SUCCESS);
  EXPECT_EQ(max, val);
}

/// Custom parameter type
typedef struct {void *unused; int32_t i;} custom_param_t;

/// Custom parameter getter.
static param_value_union_t
custom_param_getter(enum MAV_PARAM_TYPE type, const char *id, void *data) {
  return (param_value_union_t){.param_int32 = ((custom_param_t*) data)->i};
}

/// Custom parameter setter.
static ret_status_t
custom_param_setter(enum MAV_PARAM_TYPE type, const char *id,
		     void *data, param_value_union_t new_value) {
  ((custom_param_t*) data)->i = new_value.param_int32;
  return STATUS_SUCCESS;
}

/// Test custom parameter getter/setter.
TEST(ParamTest, CustomGetSet) {
  custom_param_t val = {.unused = NULL, .i=3};
  param_t param = {
    .type = MAV_PARAM_TYPE_INT32,
    .id = {0},
    .data = &val,
    .getter = &custom_param_getter,
    .setter = &custom_param_setter
  };
  EXPECT_EQ(param_get(&param).param_int32, val.i);

  param_value_union_t new_value = {.param_int32 = -1000};
  EXPECT_EQ(param_set(&param, new_value), STATUS_SUCCESS);
  EXPECT_EQ(val.i, new_value.param_int32);
}
