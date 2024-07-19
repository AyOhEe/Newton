using Godot;
using System;

public partial class ShoulderJointPositioning : Node
{
	[Export] public BodySolver VRBodySolver;
	[Export] public RigidBody3D CoreRB;
	[Export] public Node3D ChestMount;
	[Export] public PinJointController ShoulderJointController;
	[Export] public bool isLeftShoulder;

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		Transform3D localPose = GetPose();
		Transform3D relativePose = GetChestPose().Inverse() * localPose;
		ShoulderJointController.JointPosA = CoreRB.GlobalTransform.Inverse() * ChestMount.GlobalTransform * relativePose.Origin;
	}
	private Transform3D GetPose()
	{
		if (isLeftShoulder)
		{
			return new Transform3D(VRBodySolver.GetLShoulderBas(), VRBodySolver.GetLShoulderPos());
		}
		else
		{
            return new Transform3D(VRBodySolver.GetRShoulderBas(), VRBodySolver.GetRShoulderPos());
        }
	}
	private Transform3D GetChestPose()
	{
		return new Transform3D(VRBodySolver.GetChestBas(), VRBodySolver.GetChestPos());
	}
}
