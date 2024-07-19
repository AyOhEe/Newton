using Godot;
using System;

public partial class MatchArmRotation : Node
{
	[Export] public BodySolver VRBodySolver;
	[Export] public RigidBody3D RB;
    [Export] public Node3D BoneMount;
	[Export] public bool isLeftArm;
	[Export] public bool isForearm;

    [ExportCategory("Angular motion settings")]
    [Export] private float _AngularApproachTime = 0.05f;
    [Export] private float _MaxTorque = 1;

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
        float fdelta = (float)delta;

        //convert the current and desired rotations into quaternsions, then get the difference
        Quaternion currentRot = RB.GlobalBasis.GetRotationQuaternion();
        Quaternion desiredRot = GetTargetBasis().GetRotationQuaternion() * BoneMount.Basis.Inverse().GetRotationQuaternion();
        Quaternion deltaQ = desiredRot * currentRot.Inverse();

        //use the smallest rotation possible (one quaternion can represent two possible rotations)
        if (deltaQ.GetAngle() > Mathf.Pi)
        {
            deltaQ = -deltaQ;
        }

        //use this to calculate the axis and angle of rotation
        Vector3 desiredAngVel = (deltaQ.GetAxis() * deltaQ.GetAngle()) / _AngularApproachTime;
        Vector3 desiredAngMomentum = RB.GetInverseInertiaTensor().Inverse() * desiredAngVel;
        Vector3 currentAngMomentum = RB.GetInverseInertiaTensor().Inverse() * RB.AngularVelocity;
        desiredAngMomentum = (desiredAngMomentum - currentAngMomentum).LimitLength(fdelta * _MaxTorque) + currentAngMomentum;
        RB.AngularVelocity = RB.GetInverseInertiaTensor() * desiredAngMomentum;
    }

	private Basis GetTargetBasis() 
	{
		if (isForearm)
		{
			return GetForearmBasis();
		}
		else
		{
			return GetArmBasis();
		}
	}
	private Basis GetForearmBasis()
	{
		if (isLeftArm)
		{
			return VRBodySolver.GetLElbowBas();
		}
		else
		{
			return VRBodySolver.GetRElbowBas();
		}
	}
    private Basis GetArmBasis()
    {
        if (isLeftArm)
        {
            return VRBodySolver.GetLShoulderBas();
        }
        else
        {
            return VRBodySolver.GetRShoulderBas();
        }
    }
}
