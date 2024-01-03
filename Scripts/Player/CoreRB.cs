using Godot;
using System;

public partial class CoreRB : RigidBody3D
{
	[Export] private Node3D _ChestMount;
	[Export] private CameraRig _CameraRig;
	[Export] private BodySolver _BodySolver;

	public override void _Process(double delta)
	{
		//position the CameraRig such that the chest matches the chest mount
		Vector3 chestPos = _CameraRig.GlobalBasis * _BodySolver.GetChestPos();
		Vector3 camRigPos = _ChestMount.GlobalPosition - chestPos;
		_CameraRig.GlobalPosition = camRigPos;
	}

    public override void _PhysicsProcess(double delta)
    {
		//match the core's rotation with the body direction
		Rotation = (_CameraRig.GlobalBasis * _BodySolver.GetBodyDirection()).GetEuler();
    }
}
