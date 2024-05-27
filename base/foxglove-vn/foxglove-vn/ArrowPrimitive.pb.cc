// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ArrowPrimitive.proto

#include "ArrowPrimitive.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace foxglove {
PROTOBUF_CONSTEXPR ArrowPrimitive::ArrowPrimitive(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.pose_)*/nullptr
  , /*decltype(_impl_.color_)*/nullptr
  , /*decltype(_impl_.shaft_length_)*/0
  , /*decltype(_impl_.shaft_diameter_)*/0
  , /*decltype(_impl_.head_length_)*/0
  , /*decltype(_impl_.head_diameter_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct ArrowPrimitiveDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ArrowPrimitiveDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ArrowPrimitiveDefaultTypeInternal() {}
  union {
    ArrowPrimitive _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ArrowPrimitiveDefaultTypeInternal _ArrowPrimitive_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_ArrowPrimitive_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_ArrowPrimitive_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_ArrowPrimitive_2eproto = nullptr;

const uint32_t TableStruct_ArrowPrimitive_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.pose_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.shaft_length_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.shaft_diameter_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.head_length_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.head_diameter_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ArrowPrimitive, _impl_.color_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::foxglove::ArrowPrimitive)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::foxglove::_ArrowPrimitive_default_instance_._instance,
};

const char descriptor_table_protodef_ArrowPrimitive_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\024ArrowPrimitive.proto\022\010foxglove\032\013Color."
  "proto\032\nPose.proto\"\250\001\n\016ArrowPrimitive\022\034\n\004"
  "pose\030\001 \001(\0132\016.foxglove.Pose\022\024\n\014shaft_leng"
  "th\030\002 \001(\001\022\026\n\016shaft_diameter\030\003 \001(\001\022\023\n\013head"
  "_length\030\004 \001(\001\022\025\n\rhead_diameter\030\005 \001(\001\022\036\n\005"
  "color\030\006 \001(\0132\017.foxglove.Colorb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_ArrowPrimitive_2eproto_deps[2] = {
  &::descriptor_table_Color_2eproto,
  &::descriptor_table_Pose_2eproto,
};
static ::_pbi::once_flag descriptor_table_ArrowPrimitive_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_ArrowPrimitive_2eproto = {
    false, false, 236, descriptor_table_protodef_ArrowPrimitive_2eproto,
    "ArrowPrimitive.proto",
    &descriptor_table_ArrowPrimitive_2eproto_once, descriptor_table_ArrowPrimitive_2eproto_deps, 2, 1,
    schemas, file_default_instances, TableStruct_ArrowPrimitive_2eproto::offsets,
    file_level_metadata_ArrowPrimitive_2eproto, file_level_enum_descriptors_ArrowPrimitive_2eproto,
    file_level_service_descriptors_ArrowPrimitive_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_ArrowPrimitive_2eproto_getter() {
  return &descriptor_table_ArrowPrimitive_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_ArrowPrimitive_2eproto(&descriptor_table_ArrowPrimitive_2eproto);
namespace foxglove {

// ===================================================================

class ArrowPrimitive::_Internal {
 public:
  static const ::foxglove::Pose& pose(const ArrowPrimitive* msg);
  static const ::foxglove::Color& color(const ArrowPrimitive* msg);
};

const ::foxglove::Pose&
ArrowPrimitive::_Internal::pose(const ArrowPrimitive* msg) {
  return *msg->_impl_.pose_;
}
const ::foxglove::Color&
ArrowPrimitive::_Internal::color(const ArrowPrimitive* msg) {
  return *msg->_impl_.color_;
}
void ArrowPrimitive::clear_pose() {
  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
}
void ArrowPrimitive::clear_color() {
  if (GetArenaForAllocation() == nullptr && _impl_.color_ != nullptr) {
    delete _impl_.color_;
  }
  _impl_.color_ = nullptr;
}
ArrowPrimitive::ArrowPrimitive(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:foxglove.ArrowPrimitive)
}
ArrowPrimitive::ArrowPrimitive(const ArrowPrimitive& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  ArrowPrimitive* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.color_){nullptr}
    , decltype(_impl_.shaft_length_){}
    , decltype(_impl_.shaft_diameter_){}
    , decltype(_impl_.head_length_){}
    , decltype(_impl_.head_diameter_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_pose()) {
    _this->_impl_.pose_ = new ::foxglove::Pose(*from._impl_.pose_);
  }
  if (from._internal_has_color()) {
    _this->_impl_.color_ = new ::foxglove::Color(*from._impl_.color_);
  }
  ::memcpy(&_impl_.shaft_length_, &from._impl_.shaft_length_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.head_diameter_) -
    reinterpret_cast<char*>(&_impl_.shaft_length_)) + sizeof(_impl_.head_diameter_));
  // @@protoc_insertion_point(copy_constructor:foxglove.ArrowPrimitive)
}

inline void ArrowPrimitive::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.color_){nullptr}
    , decltype(_impl_.shaft_length_){0}
    , decltype(_impl_.shaft_diameter_){0}
    , decltype(_impl_.head_length_){0}
    , decltype(_impl_.head_diameter_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

ArrowPrimitive::~ArrowPrimitive() {
  // @@protoc_insertion_point(destructor:foxglove.ArrowPrimitive)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void ArrowPrimitive::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.pose_;
  if (this != internal_default_instance()) delete _impl_.color_;
}

void ArrowPrimitive::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void ArrowPrimitive::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.ArrowPrimitive)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.color_ != nullptr) {
    delete _impl_.color_;
  }
  _impl_.color_ = nullptr;
  ::memset(&_impl_.shaft_length_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.head_diameter_) -
      reinterpret_cast<char*>(&_impl_.shaft_length_)) + sizeof(_impl_.head_diameter_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ArrowPrimitive::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .foxglove.Pose pose = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_pose(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // double shaft_length = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          _impl_.shaft_length_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double shaft_diameter = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          _impl_.shaft_diameter_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double head_length = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 33)) {
          _impl_.head_length_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double head_diameter = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 41)) {
          _impl_.head_diameter_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // .foxglove.Color color = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr = ctx->ParseMessage(_internal_mutable_color(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* ArrowPrimitive::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.ArrowPrimitive)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .foxglove.Pose pose = 1;
  if (this->_internal_has_pose()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::pose(this),
        _Internal::pose(this).GetCachedSize(), target, stream);
  }

  // double shaft_length = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_length = this->_internal_shaft_length();
  uint64_t raw_shaft_length;
  memcpy(&raw_shaft_length, &tmp_shaft_length, sizeof(tmp_shaft_length));
  if (raw_shaft_length != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(2, this->_internal_shaft_length(), target);
  }

  // double shaft_diameter = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_diameter = this->_internal_shaft_diameter();
  uint64_t raw_shaft_diameter;
  memcpy(&raw_shaft_diameter, &tmp_shaft_diameter, sizeof(tmp_shaft_diameter));
  if (raw_shaft_diameter != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(3, this->_internal_shaft_diameter(), target);
  }

  // double head_length = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_length = this->_internal_head_length();
  uint64_t raw_head_length;
  memcpy(&raw_head_length, &tmp_head_length, sizeof(tmp_head_length));
  if (raw_head_length != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(4, this->_internal_head_length(), target);
  }

  // double head_diameter = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_diameter = this->_internal_head_diameter();
  uint64_t raw_head_diameter;
  memcpy(&raw_head_diameter, &tmp_head_diameter, sizeof(tmp_head_diameter));
  if (raw_head_diameter != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(5, this->_internal_head_diameter(), target);
  }

  // .foxglove.Color color = 6;
  if (this->_internal_has_color()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(6, _Internal::color(this),
        _Internal::color(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.ArrowPrimitive)
  return target;
}

size_t ArrowPrimitive::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.ArrowPrimitive)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .foxglove.Pose pose = 1;
  if (this->_internal_has_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.pose_);
  }

  // .foxglove.Color color = 6;
  if (this->_internal_has_color()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.color_);
  }

  // double shaft_length = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_length = this->_internal_shaft_length();
  uint64_t raw_shaft_length;
  memcpy(&raw_shaft_length, &tmp_shaft_length, sizeof(tmp_shaft_length));
  if (raw_shaft_length != 0) {
    total_size += 1 + 8;
  }

  // double shaft_diameter = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_diameter = this->_internal_shaft_diameter();
  uint64_t raw_shaft_diameter;
  memcpy(&raw_shaft_diameter, &tmp_shaft_diameter, sizeof(tmp_shaft_diameter));
  if (raw_shaft_diameter != 0) {
    total_size += 1 + 8;
  }

  // double head_length = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_length = this->_internal_head_length();
  uint64_t raw_head_length;
  memcpy(&raw_head_length, &tmp_head_length, sizeof(tmp_head_length));
  if (raw_head_length != 0) {
    total_size += 1 + 8;
  }

  // double head_diameter = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_diameter = this->_internal_head_diameter();
  uint64_t raw_head_diameter;
  memcpy(&raw_head_diameter, &tmp_head_diameter, sizeof(tmp_head_diameter));
  if (raw_head_diameter != 0) {
    total_size += 1 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ArrowPrimitive::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    ArrowPrimitive::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ArrowPrimitive::GetClassData() const { return &_class_data_; }


void ArrowPrimitive::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<ArrowPrimitive*>(&to_msg);
  auto& from = static_cast<const ArrowPrimitive&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.ArrowPrimitive)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_pose()) {
    _this->_internal_mutable_pose()->::foxglove::Pose::MergeFrom(
        from._internal_pose());
  }
  if (from._internal_has_color()) {
    _this->_internal_mutable_color()->::foxglove::Color::MergeFrom(
        from._internal_color());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_length = from._internal_shaft_length();
  uint64_t raw_shaft_length;
  memcpy(&raw_shaft_length, &tmp_shaft_length, sizeof(tmp_shaft_length));
  if (raw_shaft_length != 0) {
    _this->_internal_set_shaft_length(from._internal_shaft_length());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_shaft_diameter = from._internal_shaft_diameter();
  uint64_t raw_shaft_diameter;
  memcpy(&raw_shaft_diameter, &tmp_shaft_diameter, sizeof(tmp_shaft_diameter));
  if (raw_shaft_diameter != 0) {
    _this->_internal_set_shaft_diameter(from._internal_shaft_diameter());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_length = from._internal_head_length();
  uint64_t raw_head_length;
  memcpy(&raw_head_length, &tmp_head_length, sizeof(tmp_head_length));
  if (raw_head_length != 0) {
    _this->_internal_set_head_length(from._internal_head_length());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_head_diameter = from._internal_head_diameter();
  uint64_t raw_head_diameter;
  memcpy(&raw_head_diameter, &tmp_head_diameter, sizeof(tmp_head_diameter));
  if (raw_head_diameter != 0) {
    _this->_internal_set_head_diameter(from._internal_head_diameter());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ArrowPrimitive::CopyFrom(const ArrowPrimitive& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.ArrowPrimitive)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ArrowPrimitive::IsInitialized() const {
  return true;
}

void ArrowPrimitive::InternalSwap(ArrowPrimitive* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(ArrowPrimitive, _impl_.head_diameter_)
      + sizeof(ArrowPrimitive::_impl_.head_diameter_)
      - PROTOBUF_FIELD_OFFSET(ArrowPrimitive, _impl_.pose_)>(
          reinterpret_cast<char*>(&_impl_.pose_),
          reinterpret_cast<char*>(&other->_impl_.pose_));
}

::PROTOBUF_NAMESPACE_ID::Metadata ArrowPrimitive::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_ArrowPrimitive_2eproto_getter, &descriptor_table_ArrowPrimitive_2eproto_once,
      file_level_metadata_ArrowPrimitive_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::foxglove::ArrowPrimitive*
Arena::CreateMaybeMessage< ::foxglove::ArrowPrimitive >(Arena* arena) {
  return Arena::CreateMessageInternal< ::foxglove::ArrowPrimitive >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
