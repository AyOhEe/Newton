using Godot;
using System;

public partial class HeadBoneParent : Node3D
{
	[Export] private Node3D _Eyes;
    [Export] private BoneAttachment3D _HeadBone;

    public override void _Ready()
    {
        _HeadBone.Transform = _Eyes.Transform.Inverse() * _HeadBone.Transform;
    }
}
