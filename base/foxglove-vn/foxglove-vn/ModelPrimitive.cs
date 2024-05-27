// <auto-generated>
//     Generated by the protocol buffer compiler.  DO NOT EDIT!
//     source: ModelPrimitive.proto
// </auto-generated>
#pragma warning disable 1591, 0612, 3021, 8981
#region Designer generated code

using pb = global::Google.Protobuf;
using pbc = global::Google.Protobuf.Collections;
using pbr = global::Google.Protobuf.Reflection;
using scg = global::System.Collections.Generic;
namespace Foxglove {

  /// <summary>Holder for reflection information generated from ModelPrimitive.proto</summary>
  public static partial class ModelPrimitiveReflection {

    #region Descriptor
    /// <summary>File descriptor for ModelPrimitive.proto</summary>
    public static pbr::FileDescriptor Descriptor {
      get { return descriptor; }
    }
    private static pbr::FileDescriptor descriptor;

    static ModelPrimitiveReflection() {
      byte[] descriptorData = global::System.Convert.FromBase64String(
          string.Concat(
            "ChRNb2RlbFByaW1pdGl2ZS5wcm90bxIIZm94Z2xvdmUaC0NvbG9yLnByb3Rv",
            "GgpQb3NlLnByb3RvGg1WZWN0b3IzLnByb3RvIrcBCg5Nb2RlbFByaW1pdGl2",
            "ZRIcCgRwb3NlGAEgASgLMg4uZm94Z2xvdmUuUG9zZRIgCgVzY2FsZRgCIAEo",
            "CzIRLmZveGdsb3ZlLlZlY3RvcjMSHgoFY29sb3IYAyABKAsyDy5mb3hnbG92",
            "ZS5Db2xvchIWCg5vdmVycmlkZV9jb2xvchgEIAEoCBILCgN1cmwYBSABKAkS",
            "EgoKbWVkaWFfdHlwZRgGIAEoCRIMCgRkYXRhGAcgASgMYgZwcm90bzM="));
      descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,
          new pbr::FileDescriptor[] { global::Foxglove.ColorReflection.Descriptor, global::Foxglove.PoseReflection.Descriptor, global::Foxglove.Vector3Reflection.Descriptor, },
          new pbr::GeneratedClrTypeInfo(null, null, new pbr::GeneratedClrTypeInfo[] {
            new pbr::GeneratedClrTypeInfo(typeof(global::Foxglove.ModelPrimitive), global::Foxglove.ModelPrimitive.Parser, new[]{ "Pose", "Scale", "Color", "OverrideColor", "Url", "MediaType", "Data" }, null, null, null, null)
          }));
    }
    #endregion

  }
  #region Messages
  /// <summary>
  /// A primitive representing a 3D model file loaded from an external URL or embedded data
  /// </summary>
  public sealed partial class ModelPrimitive : pb::IMessage<ModelPrimitive>
  #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      , pb::IBufferMessage
  #endif
  {
    private static readonly pb::MessageParser<ModelPrimitive> _parser = new pb::MessageParser<ModelPrimitive>(() => new ModelPrimitive());
    private pb::UnknownFieldSet _unknownFields;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pb::MessageParser<ModelPrimitive> Parser { get { return _parser; } }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pbr::MessageDescriptor Descriptor {
      get { return global::Foxglove.ModelPrimitiveReflection.Descriptor.MessageTypes[0]; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    pbr::MessageDescriptor pb::IMessage.Descriptor {
      get { return Descriptor; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ModelPrimitive() {
      OnConstruction();
    }

    partial void OnConstruction();

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ModelPrimitive(ModelPrimitive other) : this() {
      pose_ = other.pose_ != null ? other.pose_.Clone() : null;
      scale_ = other.scale_ != null ? other.scale_.Clone() : null;
      color_ = other.color_ != null ? other.color_.Clone() : null;
      overrideColor_ = other.overrideColor_;
      url_ = other.url_;
      mediaType_ = other.mediaType_;
      data_ = other.data_;
      _unknownFields = pb::UnknownFieldSet.Clone(other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ModelPrimitive Clone() {
      return new ModelPrimitive(this);
    }

    /// <summary>Field number for the "pose" field.</summary>
    public const int PoseFieldNumber = 1;
    private global::Foxglove.Pose pose_;
    /// <summary>
    /// Origin of model relative to reference frame
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Foxglove.Pose Pose {
      get { return pose_; }
      set {
        pose_ = value;
      }
    }

    /// <summary>Field number for the "scale" field.</summary>
    public const int ScaleFieldNumber = 2;
    private global::Foxglove.Vector3 scale_;
    /// <summary>
    /// Scale factor to apply to the model along each axis
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Foxglove.Vector3 Scale {
      get { return scale_; }
      set {
        scale_ = value;
      }
    }

    /// <summary>Field number for the "color" field.</summary>
    public const int ColorFieldNumber = 3;
    private global::Foxglove.Color color_;
    /// <summary>
    /// Solid color to use for the whole model if `override_color` is true.
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Foxglove.Color Color {
      get { return color_; }
      set {
        color_ = value;
      }
    }

    /// <summary>Field number for the "override_color" field.</summary>
    public const int OverrideColorFieldNumber = 4;
    private bool overrideColor_;
    /// <summary>
    /// Whether to use the color specified in `color` instead of any materials embedded in the original model.
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool OverrideColor {
      get { return overrideColor_; }
      set {
        overrideColor_ = value;
      }
    }

    /// <summary>Field number for the "url" field.</summary>
    public const int UrlFieldNumber = 5;
    private string url_ = "";
    /// <summary>
    /// URL pointing to model file. One of `url` or `data` should be provided.
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public string Url {
      get { return url_; }
      set {
        url_ = pb::ProtoPreconditions.CheckNotNull(value, "value");
      }
    }

    /// <summary>Field number for the "media_type" field.</summary>
    public const int MediaTypeFieldNumber = 6;
    private string mediaType_ = "";
    /// <summary>
    /// [Media type](https://developer.mozilla.org/en-US/docs/Web/HTTP/Basics_of_HTTP/MIME_types) of embedded model (e.g. `model/gltf-binary`). Required if `data` is provided instead of `url`. Overrides the inferred media type if `url` is provided.
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public string MediaType {
      get { return mediaType_; }
      set {
        mediaType_ = pb::ProtoPreconditions.CheckNotNull(value, "value");
      }
    }

    /// <summary>Field number for the "data" field.</summary>
    public const int DataFieldNumber = 7;
    private pb::ByteString data_ = pb::ByteString.Empty;
    /// <summary>
    /// Embedded model. One of `url` or `data` should be provided. If `data` is provided, `media_type` must be set to indicate the type of the data.
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public pb::ByteString Data {
      get { return data_; }
      set {
        data_ = pb::ProtoPreconditions.CheckNotNull(value, "value");
      }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override bool Equals(object other) {
      return Equals(other as ModelPrimitive);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool Equals(ModelPrimitive other) {
      if (ReferenceEquals(other, null)) {
        return false;
      }
      if (ReferenceEquals(other, this)) {
        return true;
      }
      if (!object.Equals(Pose, other.Pose)) return false;
      if (!object.Equals(Scale, other.Scale)) return false;
      if (!object.Equals(Color, other.Color)) return false;
      if (OverrideColor != other.OverrideColor) return false;
      if (Url != other.Url) return false;
      if (MediaType != other.MediaType) return false;
      if (Data != other.Data) return false;
      return Equals(_unknownFields, other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override int GetHashCode() {
      int hash = 1;
      if (pose_ != null) hash ^= Pose.GetHashCode();
      if (scale_ != null) hash ^= Scale.GetHashCode();
      if (color_ != null) hash ^= Color.GetHashCode();
      if (OverrideColor != false) hash ^= OverrideColor.GetHashCode();
      if (Url.Length != 0) hash ^= Url.GetHashCode();
      if (MediaType.Length != 0) hash ^= MediaType.GetHashCode();
      if (Data.Length != 0) hash ^= Data.GetHashCode();
      if (_unknownFields != null) {
        hash ^= _unknownFields.GetHashCode();
      }
      return hash;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override string ToString() {
      return pb::JsonFormatter.ToDiagnosticString(this);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void WriteTo(pb::CodedOutputStream output) {
    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      output.WriteRawMessage(this);
    #else
      if (pose_ != null) {
        output.WriteRawTag(10);
        output.WriteMessage(Pose);
      }
      if (scale_ != null) {
        output.WriteRawTag(18);
        output.WriteMessage(Scale);
      }
      if (color_ != null) {
        output.WriteRawTag(26);
        output.WriteMessage(Color);
      }
      if (OverrideColor != false) {
        output.WriteRawTag(32);
        output.WriteBool(OverrideColor);
      }
      if (Url.Length != 0) {
        output.WriteRawTag(42);
        output.WriteString(Url);
      }
      if (MediaType.Length != 0) {
        output.WriteRawTag(50);
        output.WriteString(MediaType);
      }
      if (Data.Length != 0) {
        output.WriteRawTag(58);
        output.WriteBytes(Data);
      }
      if (_unknownFields != null) {
        _unknownFields.WriteTo(output);
      }
    #endif
    }

    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    void pb::IBufferMessage.InternalWriteTo(ref pb::WriteContext output) {
      if (pose_ != null) {
        output.WriteRawTag(10);
        output.WriteMessage(Pose);
      }
      if (scale_ != null) {
        output.WriteRawTag(18);
        output.WriteMessage(Scale);
      }
      if (color_ != null) {
        output.WriteRawTag(26);
        output.WriteMessage(Color);
      }
      if (OverrideColor != false) {
        output.WriteRawTag(32);
        output.WriteBool(OverrideColor);
      }
      if (Url.Length != 0) {
        output.WriteRawTag(42);
        output.WriteString(Url);
      }
      if (MediaType.Length != 0) {
        output.WriteRawTag(50);
        output.WriteString(MediaType);
      }
      if (Data.Length != 0) {
        output.WriteRawTag(58);
        output.WriteBytes(Data);
      }
      if (_unknownFields != null) {
        _unknownFields.WriteTo(ref output);
      }
    }
    #endif

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public int CalculateSize() {
      int size = 0;
      if (pose_ != null) {
        size += 1 + pb::CodedOutputStream.ComputeMessageSize(Pose);
      }
      if (scale_ != null) {
        size += 1 + pb::CodedOutputStream.ComputeMessageSize(Scale);
      }
      if (color_ != null) {
        size += 1 + pb::CodedOutputStream.ComputeMessageSize(Color);
      }
      if (OverrideColor != false) {
        size += 1 + 1;
      }
      if (Url.Length != 0) {
        size += 1 + pb::CodedOutputStream.ComputeStringSize(Url);
      }
      if (MediaType.Length != 0) {
        size += 1 + pb::CodedOutputStream.ComputeStringSize(MediaType);
      }
      if (Data.Length != 0) {
        size += 1 + pb::CodedOutputStream.ComputeBytesSize(Data);
      }
      if (_unknownFields != null) {
        size += _unknownFields.CalculateSize();
      }
      return size;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void MergeFrom(ModelPrimitive other) {
      if (other == null) {
        return;
      }
      if (other.pose_ != null) {
        if (pose_ == null) {
          Pose = new global::Foxglove.Pose();
        }
        Pose.MergeFrom(other.Pose);
      }
      if (other.scale_ != null) {
        if (scale_ == null) {
          Scale = new global::Foxglove.Vector3();
        }
        Scale.MergeFrom(other.Scale);
      }
      if (other.color_ != null) {
        if (color_ == null) {
          Color = new global::Foxglove.Color();
        }
        Color.MergeFrom(other.Color);
      }
      if (other.OverrideColor != false) {
        OverrideColor = other.OverrideColor;
      }
      if (other.Url.Length != 0) {
        Url = other.Url;
      }
      if (other.MediaType.Length != 0) {
        MediaType = other.MediaType;
      }
      if (other.Data.Length != 0) {
        Data = other.Data;
      }
      _unknownFields = pb::UnknownFieldSet.MergeFrom(_unknownFields, other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void MergeFrom(pb::CodedInputStream input) {
    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      input.ReadRawMessage(this);
    #else
      uint tag;
      while ((tag = input.ReadTag()) != 0) {
        switch(tag) {
          default:
            _unknownFields = pb::UnknownFieldSet.MergeFieldFrom(_unknownFields, input);
            break;
          case 10: {
            if (pose_ == null) {
              Pose = new global::Foxglove.Pose();
            }
            input.ReadMessage(Pose);
            break;
          }
          case 18: {
            if (scale_ == null) {
              Scale = new global::Foxglove.Vector3();
            }
            input.ReadMessage(Scale);
            break;
          }
          case 26: {
            if (color_ == null) {
              Color = new global::Foxglove.Color();
            }
            input.ReadMessage(Color);
            break;
          }
          case 32: {
            OverrideColor = input.ReadBool();
            break;
          }
          case 42: {
            Url = input.ReadString();
            break;
          }
          case 50: {
            MediaType = input.ReadString();
            break;
          }
          case 58: {
            Data = input.ReadBytes();
            break;
          }
        }
      }
    #endif
    }

    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    void pb::IBufferMessage.InternalMergeFrom(ref pb::ParseContext input) {
      uint tag;
      while ((tag = input.ReadTag()) != 0) {
        switch(tag) {
          default:
            _unknownFields = pb::UnknownFieldSet.MergeFieldFrom(_unknownFields, ref input);
            break;
          case 10: {
            if (pose_ == null) {
              Pose = new global::Foxglove.Pose();
            }
            input.ReadMessage(Pose);
            break;
          }
          case 18: {
            if (scale_ == null) {
              Scale = new global::Foxglove.Vector3();
            }
            input.ReadMessage(Scale);
            break;
          }
          case 26: {
            if (color_ == null) {
              Color = new global::Foxglove.Color();
            }
            input.ReadMessage(Color);
            break;
          }
          case 32: {
            OverrideColor = input.ReadBool();
            break;
          }
          case 42: {
            Url = input.ReadString();
            break;
          }
          case 50: {
            MediaType = input.ReadString();
            break;
          }
          case 58: {
            Data = input.ReadBytes();
            break;
          }
        }
      }
    }
    #endif

  }

  #endregion

}

#endregion Designer generated code
