using Godot;
using Godot.Collections;
using System;

[GlobalClass]
public partial class BodyMesh : Node
{
    [Export] private Array<Node3D> _BoneAttachments;
    [Export] private Array<Node3D> _BoneAttachmentMounts;

    [ExportCategory("Arm segment bone parents")]
    [Export] private ArmSegmentBoneParent _LeftArmParent;
    [Export] private ArmSegmentBoneParent _LeftForearmParent;
    [Export] private ArmSegmentBoneParent _RightArmParent;
    [Export] private ArmSegmentBoneParent _RightForearmParent;

    public override void _EnterTree()
    {
        if(_BoneAttachmentMounts.Count != _BoneAttachments.Count)
        {
            GD.PrintErr("BodyMesh was not provided with equal bone attachments and bone group mounts!");
        }
    }

    public override void _Process(double delta)
    {
        PositionBones();
        SetArmSegmentLengths();
    }

    private void PositionBones()
    {
        if (_BoneAttachmentMounts.Count != _BoneAttachments.Count)
        {
            return;
        }

        for (int i = 0; i < _BoneAttachments.Count; i++)
        {
            if (_BoneAttachments[i] == null || _BoneAttachmentMounts[i] == null)
            {
                continue;
            }

            _BoneAttachments[i].GlobalTransform = _BoneAttachmentMounts[i].GlobalTransform;
        }
    }

    private void SetArmSegmentLengths()
    {
        _LeftArmParent.TargetLength = VRUserMeasurements.Arm;
        _LeftForearmParent.TargetLength = VRUserMeasurements.Forearm;
        _RightArmParent.TargetLength = VRUserMeasurements.Arm;
        _RightForearmParent.TargetLength = VRUserMeasurements.Forearm;
    }
}
