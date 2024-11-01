// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: CubePrimitive.proto

#include "CubePrimitive.pb.h"

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
PROTOBUF_CONSTEXPR CubePrimitive::CubePrimitive(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.pose_)*/nullptr
  , /*decltype(_impl_.size_)*/nullptr
  , /*decltype(_impl_.color_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct CubePrimitiveDefaultTypeInternal {
  PROTOBUF_CONSTEXPR CubePrimitiveDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~CubePrimitiveDefaultTypeInternal() {}
  union {
    CubePrimitive _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 CubePrimitiveDefaultTypeInternal _CubePrimitive_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_CubePrimitive_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_CubePrimitive_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_CubePrimitive_2eproto = nullptr;

const uint32_t TableStruct_CubePrimitive_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::foxglove::CubePrimitive, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::foxglove::CubePrimitive, _impl_.pose_),
  PROTOBUF_FIELD_OFFSET(::foxglove::CubePrimitive, _impl_.size_),
  PROTOBUF_FIELD_OFFSET(::foxglove::CubePrimitive, _impl_.color_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::foxglove::CubePrimitive)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::foxglove::_CubePrimitive_default_instance_._instance,
};

const char descriptor_table_protodef_CubePrimitive_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\023CubePrimitive.proto\022\010foxglove\032\013Color.p"
  "roto\032\nPose.proto\032\rVector3.proto\"n\n\rCubeP"
  "rimitive\022\034\n\004pose\030\001 \001(\0132\016.foxglove.Pose\022\037"
  "\n\004size\030\002 \001(\0132\021.foxglove.Vector3\022\036\n\005color"
  "\030\003 \001(\0132\017.foxglove.Colorb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_CubePrimitive_2eproto_deps[3] = {
  &::descriptor_table_Color_2eproto,
  &::descriptor_table_Pose_2eproto,
  &::descriptor_table_Vector3_2eproto,
};
static ::_pbi::once_flag descriptor_table_CubePrimitive_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_CubePrimitive_2eproto = {
    false, false, 191, descriptor_table_protodef_CubePrimitive_2eproto,
    "CubePrimitive.proto",
    &descriptor_table_CubePrimitive_2eproto_once, descriptor_table_CubePrimitive_2eproto_deps, 3, 1,
    schemas, file_default_instances, TableStruct_CubePrimitive_2eproto::offsets,
    file_level_metadata_CubePrimitive_2eproto, file_level_enum_descriptors_CubePrimitive_2eproto,
    file_level_service_descriptors_CubePrimitive_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_CubePrimitive_2eproto_getter() {
  return &descriptor_table_CubePrimitive_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_CubePrimitive_2eproto(&descriptor_table_CubePrimitive_2eproto);
namespace foxglove {

// ===================================================================

class CubePrimitive::_Internal {
 public:
  static const ::foxglove::Pose& pose(const CubePrimitive* msg);
  static const ::foxglove::Vector3& size(const CubePrimitive* msg);
  static const ::foxglove::Color& color(const CubePrimitive* msg);
};

const ::foxglove::Pose&
CubePrimitive::_Internal::pose(const CubePrimitive* msg) {
  return *msg->_impl_.pose_;
}
const ::foxglove::Vector3&
CubePrimitive::_Internal::size(const CubePrimitive* msg) {
  return *msg->_impl_.size_;
}
const ::foxglove::Color&
CubePrimitive::_Internal::color(const CubePrimitive* msg) {
  return *msg->_impl_.color_;
}
void CubePrimitive::clear_pose() {
  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
}
void CubePrimitive::clear_size() {
  if (GetArenaForAllocation() == nullptr && _impl_.size_ != nullptr) {
    delete _impl_.size_;
  }
  _impl_.size_ = nullptr;
}
void CubePrimitive::clear_color() {
  if (GetArenaForAllocation() == nullptr && _impl_.color_ != nullptr) {
    delete _impl_.color_;
  }
  _impl_.color_ = nullptr;
}
CubePrimitive::CubePrimitive(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:foxglove.CubePrimitive)
}
CubePrimitive::CubePrimitive(const CubePrimitive& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  CubePrimitive* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.size_){nullptr}
    , decltype(_impl_.color_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_pose()) {
    _this->_impl_.pose_ = new ::foxglove::Pose(*from._impl_.pose_);
  }
  if (from._internal_has_size()) {
    _this->_impl_.size_ = new ::foxglove::Vector3(*from._impl_.size_);
  }
  if (from._internal_has_color()) {
    _this->_impl_.color_ = new ::foxglove::Color(*from._impl_.color_);
  }
  // @@protoc_insertion_point(copy_constructor:foxglove.CubePrimitive)
}

inline void CubePrimitive::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.size_){nullptr}
    , decltype(_impl_.color_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

CubePrimitive::~CubePrimitive() {
  // @@protoc_insertion_point(destructor:foxglove.CubePrimitive)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void CubePrimitive::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete _impl_.pose_;
  if (this != internal_default_instance()) delete _impl_.size_;
  if (this != internal_default_instance()) delete _impl_.color_;
}

void CubePrimitive::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void CubePrimitive::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.CubePrimitive)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.size_ != nullptr) {
    delete _impl_.size_;
  }
  _impl_.size_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.color_ != nullptr) {
    delete _impl_.color_;
  }
  _impl_.color_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CubePrimitive::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
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
      // .foxglove.Vector3 size = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_size(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .foxglove.Color color = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
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

uint8_t* CubePrimitive::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.CubePrimitive)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .foxglove.Pose pose = 1;
  if (this->_internal_has_pose()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::pose(this),
        _Internal::pose(this).GetCachedSize(), target, stream);
  }

  // .foxglove.Vector3 size = 2;
  if (this->_internal_has_size()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::size(this),
        _Internal::size(this).GetCachedSize(), target, stream);
  }

  // .foxglove.Color color = 3;
  if (this->_internal_has_color()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::color(this),
        _Internal::color(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.CubePrimitive)
  return target;
}

size_t CubePrimitive::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.CubePrimitive)
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

  // .foxglove.Vector3 size = 2;
  if (this->_internal_has_size()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.size_);
  }

  // .foxglove.Color color = 3;
  if (this->_internal_has_color()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.color_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CubePrimitive::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    CubePrimitive::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CubePrimitive::GetClassData() const { return &_class_data_; }


void CubePrimitive::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<CubePrimitive*>(&to_msg);
  auto& from = static_cast<const CubePrimitive&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.CubePrimitive)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_pose()) {
    _this->_internal_mutable_pose()->::foxglove::Pose::MergeFrom(
        from._internal_pose());
  }
  if (from._internal_has_size()) {
    _this->_internal_mutable_size()->::foxglove::Vector3::MergeFrom(
        from._internal_size());
  }
  if (from._internal_has_color()) {
    _this->_internal_mutable_color()->::foxglove::Color::MergeFrom(
        from._internal_color());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CubePrimitive::CopyFrom(const CubePrimitive& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.CubePrimitive)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CubePrimitive::IsInitialized() const {
  return true;
}

void CubePrimitive::InternalSwap(CubePrimitive* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CubePrimitive, _impl_.color_)
      + sizeof(CubePrimitive::_impl_.color_)
      - PROTOBUF_FIELD_OFFSET(CubePrimitive, _impl_.pose_)>(
          reinterpret_cast<char*>(&_impl_.pose_),
          reinterpret_cast<char*>(&other->_impl_.pose_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CubePrimitive::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_CubePrimitive_2eproto_getter, &descriptor_table_CubePrimitive_2eproto_once,
      file_level_metadata_CubePrimitive_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::foxglove::CubePrimitive*
Arena::CreateMaybeMessage< ::foxglove::CubePrimitive >(Arena* arena) {
  return Arena::CreateMessageInternal< ::foxglove::CubePrimitive >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
