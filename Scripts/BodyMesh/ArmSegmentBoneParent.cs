using Godot;
using System;

[GlobalClass]
public partial class ArmSegmentBoneParent : Node3D
{
	[Export] private Node3D SegmentStart;
	[Export] private Node3D SegmentEnd;
    [Export] private BoneAttachment3D Bone;
    [Export] private Vector3 RollAxis = Vector3.Back;
    [Export] private Vector3 ScaleAxis = Vector3.Back;
	[Export] public float TargetLength;
	[Export] public float Roll;

	private float _SegmentLength;
    private Basis _InitialRotation;

    public override void _Ready()
    {
		_SegmentLength = SegmentStart.GlobalPosition.DistanceTo(SegmentEnd.GlobalPosition);
        _InitialRotation = Bone.Basis;
    }

    public override void _Process(double delta)
    {
        float scaleFactor = (TargetLength / _SegmentLength) - 1;
        if(TargetLength == 0)
        {
            //a target length of 0 implies "match the segment"
            scaleFactor = 0;
        }


        Bone.Basis = _InitialRotation
            * Basis.FromEuler(RollAxis * Roll)
            * Basis.FromScale(Vector3.One + (ScaleAxis * scaleFactor));

        Vector3 localSegmentStart = Bone.Basis * SegmentStart.Position;
        Vector3 localSegmentEnd = Bone.Basis * SegmentEnd.Position;

        Bone.Position = -(localSegmentStart.Lerp(localSegmentEnd, 0.5f));
    }
}
