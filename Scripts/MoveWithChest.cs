using Godot;
using Godot.Collections;
using System;

public partial class MoveWithChest : Node
{
	[Export] private BodySolver _BodySolver;
	[Export] private CameraRig _CameraRig;
	[Export] private Array<RigidBody3D> _RigRigidbodies;


	private Vector3 _LastChestPos = Vector3.Zero;


	public override void _PhysicsProcess(double delta)
	{
		//get the distance the chest moved on the XZ plane this tick
		Vector3 chestPos = _CameraRig.GlobalBasis * _BodySolver.GetChestPos();
		Vector3 chestDelta = chestPos - _LastChestPos;
		chestDelta.Y = 0;


		//move all rig rigidbodies by this change
		foreach(RigidBody3D rb in _RigRigidbodies)
		{
			rb.GlobalPosition += chestDelta;
		}


		//store the chest position for the next tick
        _LastChestPos = chestPos;
	}
}
