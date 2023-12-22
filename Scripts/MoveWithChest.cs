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
		//the chest position is first calculated relative to the camera rig,
		//only taking into account scaling
		Vector3 chestPos = Scale(_BodySolver.GetChestPos(), _CameraRig.GlobalBasis.Scale);
		Vector3 chestDelta = chestPos - _LastChestPos;
		chestDelta.Y = 0;
        //once the difference has been obtained, it is then rotated to match the camera rig
        //if we do not do this, turning occurs around the camera rig origin instead of around the camera
        chestDelta = _CameraRig.GlobalBasis.Orthonormalized() * chestDelta;


		//move all rig rigidbodies by this change
		foreach(RigidBody3D rb in _RigRigidbodies)
		{
			rb.GlobalPosition += chestDelta;
		}


		//store the chest position for the next tick
        _LastChestPos = chestPos;
	}

	private Vector3 Scale(Vector3 A, Vector3 B)
	{
		A.Deconstruct(out float ax, out float ay, out float az);
		B.Deconstruct(out float bx, out float by, out float bz);
		return new Vector3(ax * bx, ay * by, az * bz);
	}
}
