using Godot;
using System;

public partial class RescaleArmSegment : Node
{
    [Export] PinJointController JointWhereA;
    [Export] PinJointController JointWhereB;
    [Export] CollisionShape3D CollisionShape;
    [Export] MeshInstance3D MeshInstance;

    [Export] bool isForearm;
    [Export] float radiusRatio = 0.167f;

    public override void _EnterTree()
    {
        VRUserMeasurements._Instance.MeasurementsChange += HandleMeasurementsChange;
        HandleMeasurementsChange();
    }
    public override void _ExitTree()
    {
        VRUserMeasurements._Instance.MeasurementsChange -= HandleMeasurementsChange;
    }

    private void HandleMeasurementsChange()
    {
        if(CollisionShape == null || !(CollisionShape.Shape is CapsuleShape3D))
        {
            return;
        }

        float length = getLengthMeasurement();
        CapsuleShape3D capsuleShape = (CapsuleShape3D)CollisionShape.Shape;

        capsuleShape.Radius = length * radiusRatio;
        capsuleShape.Height = length + (capsuleShape.Radius * 2);
        if(MeshInstance != null && MeshInstance.Mesh is CapsuleMesh)
        {
            CapsuleMesh mesh = (CapsuleMesh)(MeshInstance.Mesh);
            mesh.Radius = capsuleShape.Radius;
            mesh.Height = capsuleShape.Height;
        }

        JointWhereA.JointPosA = new Vector3(0,  (length / 2), 0);
        JointWhereB.JointPosB = new Vector3(0, -(length / 2), 0);
    }

    private float getLengthMeasurement()
    {
        if (isForearm)
        {
            return VRUserMeasurements.Forearm;
        }
        else
        {
            return VRUserMeasurements.Arm;
        }
    }
}
