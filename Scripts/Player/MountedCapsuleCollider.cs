using Godot;
using System;

public partial class MountedCapsuleCollider : CollisionShape3D
{
	[Export] private Node3D BodyA;
    [Export] private Vector3 PointA;
    [Export] private Node3D BodyB;
	[Export] private Vector3 PointB;
	[Export] private float Radius;

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		Vector3 globalPointA = BodyA.GlobalTransform * PointA;
        Vector3 globalPointB = BodyB.GlobalTransform * PointB;

		((CapsuleShape3D)Shape).Height = (Radius * 2) + globalPointA.DistanceTo(globalPointB);
		((CapsuleShape3D)Shape).Radius = Radius;

		GlobalPosition = globalPointA.Lerp(globalPointB, 0.5f);
		GlobalBasis = Basis.LookingAt(Vector3.Forward, globalPointA.DirectionTo(globalPointB));
    }
}
