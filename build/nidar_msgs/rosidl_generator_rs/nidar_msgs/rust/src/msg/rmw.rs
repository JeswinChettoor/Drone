#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "nidar_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nidar_msgs__msg__PersonDetection() -> *const std::ffi::c_void;
}

#[link(name = "nidar_msgs__rosidl_generator_c")]
extern "C" {
    fn nidar_msgs__msg__PersonDetection__init(msg: *mut PersonDetection) -> bool;
    fn nidar_msgs__msg__PersonDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PersonDetection>, size: usize) -> bool;
    fn nidar_msgs__msg__PersonDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PersonDetection>);
    fn nidar_msgs__msg__PersonDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PersonDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<PersonDetection>) -> bool;
}

// Corresponds to nidar_msgs__msg__PersonDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PersonDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x_min: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y_min: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x_max: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y_max: f32,

}



impl Default for PersonDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nidar_msgs__msg__PersonDetection__init(&mut msg as *mut _) {
        panic!("Call to nidar_msgs__msg__PersonDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PersonDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PersonDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PersonDetection where Self: Sized {
  const TYPE_NAME: &'static str = "nidar_msgs/msg/PersonDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nidar_msgs__msg__PersonDetection() }
  }
}


#[link(name = "nidar_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__nidar_msgs__msg__PersonDetectionArray() -> *const std::ffi::c_void;
}

#[link(name = "nidar_msgs__rosidl_generator_c")]
extern "C" {
    fn nidar_msgs__msg__PersonDetectionArray__init(msg: *mut PersonDetectionArray) -> bool;
    fn nidar_msgs__msg__PersonDetectionArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PersonDetectionArray>, size: usize) -> bool;
    fn nidar_msgs__msg__PersonDetectionArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PersonDetectionArray>);
    fn nidar_msgs__msg__PersonDetectionArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PersonDetectionArray>, out_seq: *mut rosidl_runtime_rs::Sequence<PersonDetectionArray>) -> bool;
}

// Corresponds to nidar_msgs__msg__PersonDetectionArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PersonDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::PersonDetection>,

}



impl Default for PersonDetectionArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !nidar_msgs__msg__PersonDetectionArray__init(&mut msg as *mut _) {
        panic!("Call to nidar_msgs__msg__PersonDetectionArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PersonDetectionArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetectionArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetectionArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { nidar_msgs__msg__PersonDetectionArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PersonDetectionArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PersonDetectionArray where Self: Sized {
  const TYPE_NAME: &'static str = "nidar_msgs/msg/PersonDetectionArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__nidar_msgs__msg__PersonDetectionArray() }
  }
}


