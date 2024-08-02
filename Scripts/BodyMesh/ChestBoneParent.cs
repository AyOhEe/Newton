using Godot;
using System;

public partial class ChestBoneParent : Node3D
{
    [Export] private Node3D _Chest;
    [Export] private BoneAttachment3D _ChestBone;

    public override void _Ready()
    {
        _ChestBone.Transform = _Chest.Transform.Inverse() * _ChestBone.Transform;
    }
}
