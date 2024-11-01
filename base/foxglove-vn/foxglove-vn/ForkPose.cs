// <auto-generated>
//     Generated by the protocol buffer compiler.  DO NOT EDIT!
//     source: ForkPose.proto
// </auto-generated>
#pragma warning disable 1591, 0612, 3021, 8981
#region Designer generated code

using pb = global::Google.Protobuf;
using pbc = global::Google.Protobuf.Collections;
using pbr = global::Google.Protobuf.Reflection;
using scg = global::System.Collections.Generic;
namespace Foxglove {

  /// <summary>Holder for reflection information generated from ForkPose.proto</summary>
  public static partial class ForkPoseReflection {

    #region Descriptor
    /// <summary>File descriptor for ForkPose.proto</summary>
    public static pbr::FileDescriptor Descriptor {
      get { return descriptor; }
    }
    private static pbr::FileDescriptor descriptor;

    static ForkPoseReflection() {
      byte[] descriptorData = global::System.Convert.FromBase64String(
          string.Concat(
            "Cg5Gb3JrUG9zZS5wcm90bxIIZm94Z2xvdmUaEFF1YXRlcm5pb24ucHJvdG8a",
            "DVZlY3RvcjMucHJvdG8i3AEKCEZvcmtQb3NlEgkKAXgYASABKAISEQoJeF9p",
            "c3ZhbGlkGAIgASgIEgkKAXkYAyABKAISEQoJeV9pc3ZhbGlkGAQgASgIEgkK",
            "AXoYBSABKAISEQoJel9pc3ZhbGlkGAYgASgIEgkKAXIYByABKAISEQoJcl9p",
            "c3ZhbGlkGAggASgIEgkKAXAYCSABKAISEQoJcF9pc3ZhbGlkGAogASgIEgkK",
            "AWEYCyABKAISEQoJYV9pc3ZhbGlkGAwgASgIEgkKAWMYDSABKAISEQoJY19p",
            "c3ZhbGlkGA4gASgIYgZwcm90bzM="));
      descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,
          new pbr::FileDescriptor[] { global::Foxglove.QuaternionReflection.Descriptor, global::Foxglove.Vector3Reflection.Descriptor, },
          new pbr::GeneratedClrTypeInfo(null, null, new pbr::GeneratedClrTypeInfo[] {
            new pbr::GeneratedClrTypeInfo(typeof(global::Foxglove.ForkPose), global::Foxglove.ForkPose.Parser, new[]{ "X", "XIsvalid", "Y", "YIsvalid", "Z", "ZIsvalid", "R", "RIsvalid", "P", "PIsvalid", "A", "AIsvalid", "C", "CIsvalid" }, null, null, null, null)
          }));
    }
    #endregion

  }
  #region Messages
  /// <summary>
  /// A position and orientation for an object or reference frame in 3D space
  /// </summary>
  public sealed partial class ForkPose : pb::IMessage<ForkPose>
  #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      , pb::IBufferMessage
  #endif
  {
    private static readonly pb::MessageParser<ForkPose> _parser = new pb::MessageParser<ForkPose>(() => new ForkPose());
    private pb::UnknownFieldSet _unknownFields;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pb::MessageParser<ForkPose> Parser { get { return _parser; } }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pbr::MessageDescriptor Descriptor {
      get { return global::Foxglove.ForkPoseReflection.Descriptor.MessageTypes[0]; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    pbr::MessageDescriptor pb::IMessage.Descriptor {
      get { return Descriptor; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ForkPose() {
      OnConstruction();
    }

    partial void OnConstruction();

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ForkPose(ForkPose other) : this() {
      x_ = other.x_;
      xIsvalid_ = other.xIsvalid_;
      y_ = other.y_;
      yIsvalid_ = other.yIsvalid_;
      z_ = other.z_;
      zIsvalid_ = other.zIsvalid_;
      r_ = other.r_;
      rIsvalid_ = other.rIsvalid_;
      p_ = other.p_;
      pIsvalid_ = other.pIsvalid_;
      a_ = other.a_;
      aIsvalid_ = other.aIsvalid_;
      c_ = other.c_;
      cIsvalid_ = other.cIsvalid_;
      _unknownFields = pb::UnknownFieldSet.Clone(other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public ForkPose Clone() {
      return new ForkPose(this);
    }

    /// <summary>Field number for the "x" field.</summary>
    public const int XFieldNumber = 1;
    private float x_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float X {
      get { return x_; }
      set {
        x_ = value;
      }
    }

    /// <summary>Field number for the "x_isvalid" field.</summary>
    public const int XIsvalidFieldNumber = 2;
    private bool xIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool XIsvalid {
      get { return xIsvalid_; }
      set {
        xIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "y" field.</summary>
    public const int YFieldNumber = 3;
    private float y_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float Y {
      get { return y_; }
      set {
        y_ = value;
      }
    }

    /// <summary>Field number for the "y_isvalid" field.</summary>
    public const int YIsvalidFieldNumber = 4;
    private bool yIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool YIsvalid {
      get { return yIsvalid_; }
      set {
        yIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "z" field.</summary>
    public const int ZFieldNumber = 5;
    private float z_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float Z {
      get { return z_; }
      set {
        z_ = value;
      }
    }

    /// <summary>Field number for the "z_isvalid" field.</summary>
    public const int ZIsvalidFieldNumber = 6;
    private bool zIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool ZIsvalid {
      get { return zIsvalid_; }
      set {
        zIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "r" field.</summary>
    public const int RFieldNumber = 7;
    private float r_;
    /// <summary>
    ///roll
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float R {
      get { return r_; }
      set {
        r_ = value;
      }
    }

    /// <summary>Field number for the "r_isvalid" field.</summary>
    public const int RIsvalidFieldNumber = 8;
    private bool rIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool RIsvalid {
      get { return rIsvalid_; }
      set {
        rIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "p" field.</summary>
    public const int PFieldNumber = 9;
    private float p_;
    /// <summary>
    ///pitch
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float P {
      get { return p_; }
      set {
        p_ = value;
      }
    }

    /// <summary>Field number for the "p_isvalid" field.</summary>
    public const int PIsvalidFieldNumber = 10;
    private bool pIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool PIsvalid {
      get { return pIsvalid_; }
      set {
        pIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "a" field.</summary>
    public const int AFieldNumber = 11;
    private float a_;
    /// <summary>
    ///yaw
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float A {
      get { return a_; }
      set {
        a_ = value;
      }
    }

    /// <summary>Field number for the "a_isvalid" field.</summary>
    public const int AIsvalidFieldNumber = 12;
    private bool aIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool AIsvalid {
      get { return aIsvalid_; }
      set {
        aIsvalid_ = value;
      }
    }

    /// <summary>Field number for the "c" field.</summary>
    public const int CFieldNumber = 13;
    private float c_;
    /// <summary>
    ///c
    /// </summary>
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public float C {
      get { return c_; }
      set {
        c_ = value;
      }
    }

    /// <summary>Field number for the "c_isvalid" field.</summary>
    public const int CIsvalidFieldNumber = 14;
    private bool cIsvalid_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool CIsvalid {
      get { return cIsvalid_; }
      set {
        cIsvalid_ = value;
      }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override bool Equals(object other) {
      return Equals(other as ForkPose);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool Equals(ForkPose other) {
      if (ReferenceEquals(other, null)) {
        return false;
      }
      if (ReferenceEquals(other, this)) {
        return true;
      }
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(X, other.X)) return false;
      if (XIsvalid != other.XIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(Y, other.Y)) return false;
      if (YIsvalid != other.YIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(Z, other.Z)) return false;
      if (ZIsvalid != other.ZIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(R, other.R)) return false;
      if (RIsvalid != other.RIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(P, other.P)) return false;
      if (PIsvalid != other.PIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(A, other.A)) return false;
      if (AIsvalid != other.AIsvalid) return false;
      if (!pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.Equals(C, other.C)) return false;
      if (CIsvalid != other.CIsvalid) return false;
      return Equals(_unknownFields, other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override int GetHashCode() {
      int hash = 1;
      if (X != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(X);
      if (XIsvalid != false) hash ^= XIsvalid.GetHashCode();
      if (Y != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(Y);
      if (YIsvalid != false) hash ^= YIsvalid.GetHashCode();
      if (Z != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(Z);
      if (ZIsvalid != false) hash ^= ZIsvalid.GetHashCode();
      if (R != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(R);
      if (RIsvalid != false) hash ^= RIsvalid.GetHashCode();
      if (P != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(P);
      if (PIsvalid != false) hash ^= PIsvalid.GetHashCode();
      if (A != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(A);
      if (AIsvalid != false) hash ^= AIsvalid.GetHashCode();
      if (C != 0F) hash ^= pbc::ProtobufEqualityComparers.BitwiseSingleEqualityComparer.GetHashCode(C);
      if (CIsvalid != false) hash ^= CIsvalid.GetHashCode();
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
      if (X != 0F) {
        output.WriteRawTag(13);
        output.WriteFloat(X);
      }
      if (XIsvalid != false) {
        output.WriteRawTag(16);
        output.WriteBool(XIsvalid);
      }
      if (Y != 0F) {
        output.WriteRawTag(29);
        output.WriteFloat(Y);
      }
      if (YIsvalid != false) {
        output.WriteRawTag(32);
        output.WriteBool(YIsvalid);
      }
      if (Z != 0F) {
        output.WriteRawTag(45);
        output.WriteFloat(Z);
      }
      if (ZIsvalid != false) {
        output.WriteRawTag(48);
        output.WriteBool(ZIsvalid);
      }
      if (R != 0F) {
        output.WriteRawTag(61);
        output.WriteFloat(R);
      }
      if (RIsvalid != false) {
        output.WriteRawTag(64);
        output.WriteBool(RIsvalid);
      }
      if (P != 0F) {
        output.WriteRawTag(77);
        output.WriteFloat(P);
      }
      if (PIsvalid != false) {
        output.WriteRawTag(80);
        output.WriteBool(PIsvalid);
      }
      if (A != 0F) {
        output.WriteRawTag(93);
        output.WriteFloat(A);
      }
      if (AIsvalid != false) {
        output.WriteRawTag(96);
        output.WriteBool(AIsvalid);
      }
      if (C != 0F) {
        output.WriteRawTag(109);
        output.WriteFloat(C);
      }
      if (CIsvalid != false) {
        output.WriteRawTag(112);
        output.WriteBool(CIsvalid);
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
      if (X != 0F) {
        output.WriteRawTag(13);
        output.WriteFloat(X);
      }
      if (XIsvalid != false) {
        output.WriteRawTag(16);
        output.WriteBool(XIsvalid);
      }
      if (Y != 0F) {
        output.WriteRawTag(29);
        output.WriteFloat(Y);
      }
      if (YIsvalid != false) {
        output.WriteRawTag(32);
        output.WriteBool(YIsvalid);
      }
      if (Z != 0F) {
        output.WriteRawTag(45);
        output.WriteFloat(Z);
      }
      if (ZIsvalid != false) {
        output.WriteRawTag(48);
        output.WriteBool(ZIsvalid);
      }
      if (R != 0F) {
        output.WriteRawTag(61);
        output.WriteFloat(R);
      }
      if (RIsvalid != false) {
        output.WriteRawTag(64);
        output.WriteBool(RIsvalid);
      }
      if (P != 0F) {
        output.WriteRawTag(77);
        output.WriteFloat(P);
      }
      if (PIsvalid != false) {
        output.WriteRawTag(80);
        output.WriteBool(PIsvalid);
      }
      if (A != 0F) {
        output.WriteRawTag(93);
        output.WriteFloat(A);
      }
      if (AIsvalid != false) {
        output.WriteRawTag(96);
        output.WriteBool(AIsvalid);
      }
      if (C != 0F) {
        output.WriteRawTag(109);
        output.WriteFloat(C);
      }
      if (CIsvalid != false) {
        output.WriteRawTag(112);
        output.WriteBool(CIsvalid);
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
      if (X != 0F) {
        size += 1 + 4;
      }
      if (XIsvalid != false) {
        size += 1 + 1;
      }
      if (Y != 0F) {
        size += 1 + 4;
      }
      if (YIsvalid != false) {
        size += 1 + 1;
      }
      if (Z != 0F) {
        size += 1 + 4;
      }
      if (ZIsvalid != false) {
        size += 1 + 1;
      }
      if (R != 0F) {
        size += 1 + 4;
      }
      if (RIsvalid != false) {
        size += 1 + 1;
      }
      if (P != 0F) {
        size += 1 + 4;
      }
      if (PIsvalid != false) {
        size += 1 + 1;
      }
      if (A != 0F) {
        size += 1 + 4;
      }
      if (AIsvalid != false) {
        size += 1 + 1;
      }
      if (C != 0F) {
        size += 1 + 4;
      }
      if (CIsvalid != false) {
        size += 1 + 1;
      }
      if (_unknownFields != null) {
        size += _unknownFields.CalculateSize();
      }
      return size;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void MergeFrom(ForkPose other) {
      if (other == null) {
        return;
      }
      if (other.X != 0F) {
        X = other.X;
      }
      if (other.XIsvalid != false) {
        XIsvalid = other.XIsvalid;
      }
      if (other.Y != 0F) {
        Y = other.Y;
      }
      if (other.YIsvalid != false) {
        YIsvalid = other.YIsvalid;
      }
      if (other.Z != 0F) {
        Z = other.Z;
      }
      if (other.ZIsvalid != false) {
        ZIsvalid = other.ZIsvalid;
      }
      if (other.R != 0F) {
        R = other.R;
      }
      if (other.RIsvalid != false) {
        RIsvalid = other.RIsvalid;
      }
      if (other.P != 0F) {
        P = other.P;
      }
      if (other.PIsvalid != false) {
        PIsvalid = other.PIsvalid;
      }
      if (other.A != 0F) {
        A = other.A;
      }
      if (other.AIsvalid != false) {
        AIsvalid = other.AIsvalid;
      }
      if (other.C != 0F) {
        C = other.C;
      }
      if (other.CIsvalid != false) {
        CIsvalid = other.CIsvalid;
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
          case 13: {
            X = input.ReadFloat();
            break;
          }
          case 16: {
            XIsvalid = input.ReadBool();
            break;
          }
          case 29: {
            Y = input.ReadFloat();
            break;
          }
          case 32: {
            YIsvalid = input.ReadBool();
            break;
          }
          case 45: {
            Z = input.ReadFloat();
            break;
          }
          case 48: {
            ZIsvalid = input.ReadBool();
            break;
          }
          case 61: {
            R = input.ReadFloat();
            break;
          }
          case 64: {
            RIsvalid = input.ReadBool();
            break;
          }
          case 77: {
            P = input.ReadFloat();
            break;
          }
          case 80: {
            PIsvalid = input.ReadBool();
            break;
          }
          case 93: {
            A = input.ReadFloat();
            break;
          }
          case 96: {
            AIsvalid = input.ReadBool();
            break;
          }
          case 109: {
            C = input.ReadFloat();
            break;
          }
          case 112: {
            CIsvalid = input.ReadBool();
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
          case 13: {
            X = input.ReadFloat();
            break;
          }
          case 16: {
            XIsvalid = input.ReadBool();
            break;
          }
          case 29: {
            Y = input.ReadFloat();
            break;
          }
          case 32: {
            YIsvalid = input.ReadBool();
            break;
          }
          case 45: {
            Z = input.ReadFloat();
            break;
          }
          case 48: {
            ZIsvalid = input.ReadBool();
            break;
          }
          case 61: {
            R = input.ReadFloat();
            break;
          }
          case 64: {
            RIsvalid = input.ReadBool();
            break;
          }
          case 77: {
            P = input.ReadFloat();
            break;
          }
          case 80: {
            PIsvalid = input.ReadBool();
            break;
          }
          case 93: {
            A = input.ReadFloat();
            break;
          }
          case 96: {
            AIsvalid = input.ReadBool();
            break;
          }
          case 109: {
            C = input.ReadFloat();
            break;
          }
          case 112: {
            CIsvalid = input.ReadBool();
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
