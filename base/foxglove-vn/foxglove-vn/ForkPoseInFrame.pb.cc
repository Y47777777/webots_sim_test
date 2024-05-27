// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ForkPoseInFrame.proto

#include "ForkPoseInFrame.pb.h"

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
PROTOBUF_CONSTEXPR ForkPoseInFrame::ForkPoseInFrame(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.frame_id_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}
  , /*decltype(_impl_.timestamp_)*/nullptr
  , /*decltype(_impl_.pose_)*/nullptr
  , /*decltype(_impl_.isvalid_)*/false
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct ForkPoseInFrameDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ForkPoseInFrameDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ForkPoseInFrameDefaultTypeInternal() {}
  union {
    ForkPoseInFrame _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ForkPoseInFrameDefaultTypeInternal _ForkPoseInFrame_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_ForkPoseInFrame_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_ForkPoseInFrame_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_ForkPoseInFrame_2eproto = nullptr;

const uint32_t TableStruct_ForkPoseInFrame_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::foxglove::ForkPoseInFrame, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::foxglove::ForkPoseInFrame, _impl_.timestamp_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ForkPoseInFrame, _impl_.frame_id_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ForkPoseInFrame, _impl_.pose_),
  PROTOBUF_FIELD_OFFSET(::foxglove::ForkPoseInFrame, _impl_.isvalid_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::foxglove::ForkPoseInFrame)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::foxglove::_ForkPoseInFrame_default_instance_._instance,
};

const char descriptor_table_protodef_ForkPoseInFrame_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\025ForkPoseInFrame.proto\022\010foxglove\032\016ForkP"
  "ose.proto\032\037google/protobuf/timestamp.pro"
  "to\"\205\001\n\017ForkPoseInFrame\022-\n\ttimestamp\030\001 \001("
  "\0132\032.google.protobuf.Timestamp\022\020\n\010frame_i"
  "d\030\002 \001(\t\022 \n\004pose\030\003 \001(\0132\022.foxglove.ForkPos"
  "e\022\017\n\007isvalid\030\004 \001(\010b\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_ForkPoseInFrame_2eproto_deps[2] = {
  &::descriptor_table_ForkPose_2eproto,
  &::descriptor_table_google_2fprotobuf_2ftimestamp_2eproto,
};
static ::_pbi::once_flag descriptor_table_ForkPoseInFrame_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_ForkPoseInFrame_2eproto = {
    false, false, 226, descriptor_table_protodef_ForkPoseInFrame_2eproto,
    "ForkPoseInFrame.proto",
    &descriptor_table_ForkPoseInFrame_2eproto_once, descriptor_table_ForkPoseInFrame_2eproto_deps, 2, 1,
    schemas, file_default_instances, TableStruct_ForkPoseInFrame_2eproto::offsets,
    file_level_metadata_ForkPoseInFrame_2eproto, file_level_enum_descriptors_ForkPoseInFrame_2eproto,
    file_level_service_descriptors_ForkPoseInFrame_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_ForkPoseInFrame_2eproto_getter() {
  return &descriptor_table_ForkPoseInFrame_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_ForkPoseInFrame_2eproto(&descriptor_table_ForkPoseInFrame_2eproto);
namespace foxglove {

// ===================================================================

class ForkPoseInFrame::_Internal {
 public:
  static const ::PROTOBUF_NAMESPACE_ID::Timestamp& timestamp(const ForkPoseInFrame* msg);
  static const ::foxglove::ForkPose& pose(const ForkPoseInFrame* msg);
};

const ::PROTOBUF_NAMESPACE_ID::Timestamp&
ForkPoseInFrame::_Internal::timestamp(const ForkPoseInFrame* msg) {
  return *msg->_impl_.timestamp_;
}
const ::foxglove::ForkPose&
ForkPoseInFrame::_Internal::pose(const ForkPoseInFrame* msg) {
  return *msg->_impl_.pose_;
}
void ForkPoseInFrame::clear_timestamp() {
  if (GetArenaForAllocation() == nullptr && _impl_.timestamp_ != nullptr) {
    delete _impl_.timestamp_;
  }
  _impl_.timestamp_ = nullptr;
}
void ForkPoseInFrame::clear_pose() {
  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
}
ForkPoseInFrame::ForkPoseInFrame(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:foxglove.ForkPoseInFrame)
}
ForkPoseInFrame::ForkPoseInFrame(const ForkPoseInFrame& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  ForkPoseInFrame* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.frame_id_){}
    , decltype(_impl_.timestamp_){nullptr}
    , decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.isvalid_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.frame_id_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.frame_id_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_frame_id().empty()) {
    _this->_impl_.frame_id_.Set(from._internal_frame_id(), 
      _this->GetArenaForAllocation());
  }
  if (from._internal_has_timestamp()) {
    _this->_impl_.timestamp_ = new ::PROTOBUF_NAMESPACE_ID::Timestamp(*from._impl_.timestamp_);
  }
  if (from._internal_has_pose()) {
    _this->_impl_.pose_ = new ::foxglove::ForkPose(*from._impl_.pose_);
  }
  _this->_impl_.isvalid_ = from._impl_.isvalid_;
  // @@protoc_insertion_point(copy_constructor:foxglove.ForkPoseInFrame)
}

inline void ForkPoseInFrame::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.frame_id_){}
    , decltype(_impl_.timestamp_){nullptr}
    , decltype(_impl_.pose_){nullptr}
    , decltype(_impl_.isvalid_){false}
    , /*decltype(_impl_._cached_size_)*/{}
  };
  _impl_.frame_id_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.frame_id_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

ForkPoseInFrame::~ForkPoseInFrame() {
  // @@protoc_insertion_point(destructor:foxglove.ForkPoseInFrame)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void ForkPoseInFrame::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.frame_id_.Destroy();
  if (this != internal_default_instance()) delete _impl_.timestamp_;
  if (this != internal_default_instance()) delete _impl_.pose_;
}

void ForkPoseInFrame::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void ForkPoseInFrame::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.ForkPoseInFrame)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.frame_id_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && _impl_.timestamp_ != nullptr) {
    delete _impl_.timestamp_;
  }
  _impl_.timestamp_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.pose_ != nullptr) {
    delete _impl_.pose_;
  }
  _impl_.pose_ = nullptr;
  _impl_.isvalid_ = false;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ForkPoseInFrame::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .google.protobuf.Timestamp timestamp = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_timestamp(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string frame_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          auto str = _internal_mutable_frame_id();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "foxglove.ForkPoseInFrame.frame_id"));
        } else
          goto handle_unusual;
        continue;
      // .foxglove.ForkPose pose = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_pose(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // bool isvalid = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 32)) {
          _impl_.isvalid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

uint8_t* ForkPoseInFrame::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.ForkPoseInFrame)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .google.protobuf.Timestamp timestamp = 1;
  if (this->_internal_has_timestamp()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::timestamp(this),
        _Internal::timestamp(this).GetCachedSize(), target, stream);
  }

  // string frame_id = 2;
  if (!this->_internal_frame_id().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_frame_id().data(), static_cast<int>(this->_internal_frame_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "foxglove.ForkPoseInFrame.frame_id");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_frame_id(), target);
  }

  // .foxglove.ForkPose pose = 3;
  if (this->_internal_has_pose()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::pose(this),
        _Internal::pose(this).GetCachedSize(), target, stream);
  }

  // bool isvalid = 4;
  if (this->_internal_isvalid() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(4, this->_internal_isvalid(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.ForkPoseInFrame)
  return target;
}

size_t ForkPoseInFrame::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.ForkPoseInFrame)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string frame_id = 2;
  if (!this->_internal_frame_id().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_frame_id());
  }

  // .google.protobuf.Timestamp timestamp = 1;
  if (this->_internal_has_timestamp()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.timestamp_);
  }

  // .foxglove.ForkPose pose = 3;
  if (this->_internal_has_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.pose_);
  }

  // bool isvalid = 4;
  if (this->_internal_isvalid() != 0) {
    total_size += 1 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ForkPoseInFrame::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    ForkPoseInFrame::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ForkPoseInFrame::GetClassData() const { return &_class_data_; }


void ForkPoseInFrame::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<ForkPoseInFrame*>(&to_msg);
  auto& from = static_cast<const ForkPoseInFrame&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.ForkPoseInFrame)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_frame_id().empty()) {
    _this->_internal_set_frame_id(from._internal_frame_id());
  }
  if (from._internal_has_timestamp()) {
    _this->_internal_mutable_timestamp()->::PROTOBUF_NAMESPACE_ID::Timestamp::MergeFrom(
        from._internal_timestamp());
  }
  if (from._internal_has_pose()) {
    _this->_internal_mutable_pose()->::foxglove::ForkPose::MergeFrom(
        from._internal_pose());
  }
  if (from._internal_isvalid() != 0) {
    _this->_internal_set_isvalid(from._internal_isvalid());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ForkPoseInFrame::CopyFrom(const ForkPoseInFrame& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.ForkPoseInFrame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ForkPoseInFrame::IsInitialized() const {
  return true;
}

void ForkPoseInFrame::InternalSwap(ForkPoseInFrame* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.frame_id_, lhs_arena,
      &other->_impl_.frame_id_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(ForkPoseInFrame, _impl_.isvalid_)
      + sizeof(ForkPoseInFrame::_impl_.isvalid_)
      - PROTOBUF_FIELD_OFFSET(ForkPoseInFrame, _impl_.timestamp_)>(
          reinterpret_cast<char*>(&_impl_.timestamp_),
          reinterpret_cast<char*>(&other->_impl_.timestamp_));
}

::PROTOBUF_NAMESPACE_ID::Metadata ForkPoseInFrame::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_ForkPoseInFrame_2eproto_getter, &descriptor_table_ForkPoseInFrame_2eproto_once,
      file_level_metadata_ForkPoseInFrame_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::foxglove::ForkPoseInFrame*
Arena::CreateMaybeMessage< ::foxglove::ForkPoseInFrame >(Arena* arena) {
  return Arena::CreateMessageInternal< ::foxglove::ForkPoseInFrame >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
