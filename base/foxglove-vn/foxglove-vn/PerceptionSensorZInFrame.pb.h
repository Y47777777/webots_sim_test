// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: PerceptionSensorZInFrame.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_PerceptionSensorZInFrame_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_PerceptionSensorZInFrame_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021012 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/timestamp.pb.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_PerceptionSensorZInFrame_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_PerceptionSensorZInFrame_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_PerceptionSensorZInFrame_2eproto;
namespace foxglove {
class PerceptionSensorZInFrame;
struct PerceptionSensorZInFrameDefaultTypeInternal;
extern PerceptionSensorZInFrameDefaultTypeInternal _PerceptionSensorZInFrame_default_instance_;
}  // namespace foxglove
PROTOBUF_NAMESPACE_OPEN
template<> ::foxglove::PerceptionSensorZInFrame* Arena::CreateMaybeMessage<::foxglove::PerceptionSensorZInFrame>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace foxglove {

// ===================================================================

class PerceptionSensorZInFrame final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:foxglove.PerceptionSensorZInFrame) */ {
 public:
  inline PerceptionSensorZInFrame() : PerceptionSensorZInFrame(nullptr) {}
  ~PerceptionSensorZInFrame() override;
  explicit PROTOBUF_CONSTEXPR PerceptionSensorZInFrame(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PerceptionSensorZInFrame(const PerceptionSensorZInFrame& from);
  PerceptionSensorZInFrame(PerceptionSensorZInFrame&& from) noexcept
    : PerceptionSensorZInFrame() {
    *this = ::std::move(from);
  }

  inline PerceptionSensorZInFrame& operator=(const PerceptionSensorZInFrame& from) {
    CopyFrom(from);
    return *this;
  }
  inline PerceptionSensorZInFrame& operator=(PerceptionSensorZInFrame&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const PerceptionSensorZInFrame& default_instance() {
    return *internal_default_instance();
  }
  static inline const PerceptionSensorZInFrame* internal_default_instance() {
    return reinterpret_cast<const PerceptionSensorZInFrame*>(
               &_PerceptionSensorZInFrame_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PerceptionSensorZInFrame& a, PerceptionSensorZInFrame& b) {
    a.Swap(&b);
  }
  inline void Swap(PerceptionSensorZInFrame* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(PerceptionSensorZInFrame* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  PerceptionSensorZInFrame* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<PerceptionSensorZInFrame>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PerceptionSensorZInFrame& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const PerceptionSensorZInFrame& from) {
    PerceptionSensorZInFrame::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(PerceptionSensorZInFrame* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "foxglove.PerceptionSensorZInFrame";
  }
  protected:
  explicit PerceptionSensorZInFrame(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kFrameIdFieldNumber = 2,
    kTimestampFieldNumber = 1,
    kRateFieldNumber = 3,
    kHeightValueFieldNumber = 4,
  };
  // string frame_id = 2;
  void clear_frame_id();
  const std::string& frame_id() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_frame_id(ArgT0&& arg0, ArgT... args);
  std::string* mutable_frame_id();
  PROTOBUF_NODISCARD std::string* release_frame_id();
  void set_allocated_frame_id(std::string* frame_id);
  private:
  const std::string& _internal_frame_id() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_frame_id(const std::string& value);
  std::string* _internal_mutable_frame_id();
  public:

  // .google.protobuf.Timestamp timestamp = 1;
  bool has_timestamp() const;
  private:
  bool _internal_has_timestamp() const;
  public:
  void clear_timestamp();
  const ::PROTOBUF_NAMESPACE_ID::Timestamp& timestamp() const;
  PROTOBUF_NODISCARD ::PROTOBUF_NAMESPACE_ID::Timestamp* release_timestamp();
  ::PROTOBUF_NAMESPACE_ID::Timestamp* mutable_timestamp();
  void set_allocated_timestamp(::PROTOBUF_NAMESPACE_ID::Timestamp* timestamp);
  private:
  const ::PROTOBUF_NAMESPACE_ID::Timestamp& _internal_timestamp() const;
  ::PROTOBUF_NAMESPACE_ID::Timestamp* _internal_mutable_timestamp();
  public:
  void unsafe_arena_set_allocated_timestamp(
      ::PROTOBUF_NAMESPACE_ID::Timestamp* timestamp);
  ::PROTOBUF_NAMESPACE_ID::Timestamp* unsafe_arena_release_timestamp();

  // double rate = 3;
  void clear_rate();
  double rate() const;
  void set_rate(double value);
  private:
  double _internal_rate() const;
  void _internal_set_rate(double value);
  public:

  // double height_value = 4;
  void clear_height_value();
  double height_value() const;
  void set_height_value(double value);
  private:
  double _internal_height_value() const;
  void _internal_set_height_value(double value);
  public:

  // @@protoc_insertion_point(class_scope:foxglove.PerceptionSensorZInFrame)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr frame_id_;
    ::PROTOBUF_NAMESPACE_ID::Timestamp* timestamp_;
    double rate_;
    double height_value_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_PerceptionSensorZInFrame_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PerceptionSensorZInFrame

// .google.protobuf.Timestamp timestamp = 1;
inline bool PerceptionSensorZInFrame::_internal_has_timestamp() const {
  return this != internal_default_instance() && _impl_.timestamp_ != nullptr;
}
inline bool PerceptionSensorZInFrame::has_timestamp() const {
  return _internal_has_timestamp();
}
inline const ::PROTOBUF_NAMESPACE_ID::Timestamp& PerceptionSensorZInFrame::_internal_timestamp() const {
  const ::PROTOBUF_NAMESPACE_ID::Timestamp* p = _impl_.timestamp_;
  return p != nullptr ? *p : reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Timestamp&>(
      ::PROTOBUF_NAMESPACE_ID::_Timestamp_default_instance_);
}
inline const ::PROTOBUF_NAMESPACE_ID::Timestamp& PerceptionSensorZInFrame::timestamp() const {
  // @@protoc_insertion_point(field_get:foxglove.PerceptionSensorZInFrame.timestamp)
  return _internal_timestamp();
}
inline void PerceptionSensorZInFrame::unsafe_arena_set_allocated_timestamp(
    ::PROTOBUF_NAMESPACE_ID::Timestamp* timestamp) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.timestamp_);
  }
  _impl_.timestamp_ = timestamp;
  if (timestamp) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:foxglove.PerceptionSensorZInFrame.timestamp)
}
inline ::PROTOBUF_NAMESPACE_ID::Timestamp* PerceptionSensorZInFrame::release_timestamp() {
  
  ::PROTOBUF_NAMESPACE_ID::Timestamp* temp = _impl_.timestamp_;
  _impl_.timestamp_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Timestamp* PerceptionSensorZInFrame::unsafe_arena_release_timestamp() {
  // @@protoc_insertion_point(field_release:foxglove.PerceptionSensorZInFrame.timestamp)
  
  ::PROTOBUF_NAMESPACE_ID::Timestamp* temp = _impl_.timestamp_;
  _impl_.timestamp_ = nullptr;
  return temp;
}
inline ::PROTOBUF_NAMESPACE_ID::Timestamp* PerceptionSensorZInFrame::_internal_mutable_timestamp() {
  
  if (_impl_.timestamp_ == nullptr) {
    auto* p = CreateMaybeMessage<::PROTOBUF_NAMESPACE_ID::Timestamp>(GetArenaForAllocation());
    _impl_.timestamp_ = p;
  }
  return _impl_.timestamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::Timestamp* PerceptionSensorZInFrame::mutable_timestamp() {
  ::PROTOBUF_NAMESPACE_ID::Timestamp* _msg = _internal_mutable_timestamp();
  // @@protoc_insertion_point(field_mutable:foxglove.PerceptionSensorZInFrame.timestamp)
  return _msg;
}
inline void PerceptionSensorZInFrame::set_allocated_timestamp(::PROTOBUF_NAMESPACE_ID::Timestamp* timestamp) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.timestamp_);
  }
  if (timestamp) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(timestamp));
    if (message_arena != submessage_arena) {
      timestamp = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, timestamp, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.timestamp_ = timestamp;
  // @@protoc_insertion_point(field_set_allocated:foxglove.PerceptionSensorZInFrame.timestamp)
}

// string frame_id = 2;
inline void PerceptionSensorZInFrame::clear_frame_id() {
  _impl_.frame_id_.ClearToEmpty();
}
inline const std::string& PerceptionSensorZInFrame::frame_id() const {
  // @@protoc_insertion_point(field_get:foxglove.PerceptionSensorZInFrame.frame_id)
  return _internal_frame_id();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void PerceptionSensorZInFrame::set_frame_id(ArgT0&& arg0, ArgT... args) {
 
 _impl_.frame_id_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:foxglove.PerceptionSensorZInFrame.frame_id)
}
inline std::string* PerceptionSensorZInFrame::mutable_frame_id() {
  std::string* _s = _internal_mutable_frame_id();
  // @@protoc_insertion_point(field_mutable:foxglove.PerceptionSensorZInFrame.frame_id)
  return _s;
}
inline const std::string& PerceptionSensorZInFrame::_internal_frame_id() const {
  return _impl_.frame_id_.Get();
}
inline void PerceptionSensorZInFrame::_internal_set_frame_id(const std::string& value) {
  
  _impl_.frame_id_.Set(value, GetArenaForAllocation());
}
inline std::string* PerceptionSensorZInFrame::_internal_mutable_frame_id() {
  
  return _impl_.frame_id_.Mutable(GetArenaForAllocation());
}
inline std::string* PerceptionSensorZInFrame::release_frame_id() {
  // @@protoc_insertion_point(field_release:foxglove.PerceptionSensorZInFrame.frame_id)
  return _impl_.frame_id_.Release();
}
inline void PerceptionSensorZInFrame::set_allocated_frame_id(std::string* frame_id) {
  if (frame_id != nullptr) {
    
  } else {
    
  }
  _impl_.frame_id_.SetAllocated(frame_id, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.frame_id_.IsDefault()) {
    _impl_.frame_id_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:foxglove.PerceptionSensorZInFrame.frame_id)
}

// double rate = 3;
inline void PerceptionSensorZInFrame::clear_rate() {
  _impl_.rate_ = 0;
}
inline double PerceptionSensorZInFrame::_internal_rate() const {
  return _impl_.rate_;
}
inline double PerceptionSensorZInFrame::rate() const {
  // @@protoc_insertion_point(field_get:foxglove.PerceptionSensorZInFrame.rate)
  return _internal_rate();
}
inline void PerceptionSensorZInFrame::_internal_set_rate(double value) {
  
  _impl_.rate_ = value;
}
inline void PerceptionSensorZInFrame::set_rate(double value) {
  _internal_set_rate(value);
  // @@protoc_insertion_point(field_set:foxglove.PerceptionSensorZInFrame.rate)
}

// double height_value = 4;
inline void PerceptionSensorZInFrame::clear_height_value() {
  _impl_.height_value_ = 0;
}
inline double PerceptionSensorZInFrame::_internal_height_value() const {
  return _impl_.height_value_;
}
inline double PerceptionSensorZInFrame::height_value() const {
  // @@protoc_insertion_point(field_get:foxglove.PerceptionSensorZInFrame.height_value)
  return _internal_height_value();
}
inline void PerceptionSensorZInFrame::_internal_set_height_value(double value) {
  
  _impl_.height_value_ = value;
}
inline void PerceptionSensorZInFrame::set_height_value(double value) {
  _internal_set_height_value(value);
  // @@protoc_insertion_point(field_set:foxglove.PerceptionSensorZInFrame.height_value)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace foxglove

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_PerceptionSensorZInFrame_2eproto
