using Godot;
using System;

public partial class LineEdgeGrabbable : Grabbable
{
    [Export] public float Length;

    public override Transform3D CalculateGrabPose(PhysbodyHand Hand)
    {
        Vector3 localPalmPoint = GlobalTransform.Inverse() * Hand.PalmGrabPoint.GlobalPosition;
        Vector3 projectedPalmPoint = new Vector3(localPalmPoint.X, 0, 0);
        //line is centered at the origin, so extents are -length/2 and length/2
        projectedPalmPoint = projectedPalmPoint.LimitLength(Length / 2);

        Vector3 grabUpDirection = Vector3.Right;
        Vector3 localUpDirection = GlobalBasis.Inverse() * Hand.PalmGrabPoint.GlobalBasis * Vector3.Up;
        if (grabUpDirection.Dot(localUpDirection) < 0)
        {
            grabUpDirection = -grabUpDirection;
        }
        Basis grabOrientation = Basis.LookingAt(Vector3.Forward, grabUpDirection);

        return ParentRigidBody.GlobalTransform.Inverse() * GlobalTransform * new Transform3D(grabOrientation, projectedPalmPoint);
    }
}
