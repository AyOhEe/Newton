using Godot;
using System;

public partial class CoreRB : RigidBody3D
{
	[Export] private Node3D _ChestMount;
	[Export] private CameraRig _CamRig;
	[Export] private BodySolver _BodySolver;

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		//position the CameraRig such that the chest matches the chest mount
		Vector3 chestPos = _CamRig.GlobalBasis * _BodySolver.GetChestPos();
		Vector3 camRigPos = _ChestMount.GlobalPosition - chestPos;
		_CamRig.GlobalPosition = camRigPos;
	}
}
