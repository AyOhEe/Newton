using Godot;
using System;

public partial class LegsJoint : Generic6DofJoint3D
{
	[Export] private Node3D _ChestMount;
    [Export] private CameraRig _CamRig;
    [Export] private BodySolver _BodySolver;
	[Export] private CollisionShape3D _LocosphereCollision;

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		float chestHeight = _CamRig.GlobalBasis.Scale.Y * _BodySolver.GetChestPos().Y;
		float locosphereRadius = ((SphereShape3D)_LocosphereCollision.Shape).Radius;
		float chestMountOffset = _ChestMount.GlobalPosition.Y - ((Node3D)_ChestMount.GetParent()).GlobalPosition.Y;

		//distance between locosphere center and chest center
		float requiredExtension = chestHeight - locosphereRadius - chestMountOffset;
		SetParamY(Param.LinearSpringEquilibriumPoint, requiredExtension);
    }
}
