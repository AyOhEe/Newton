using Godot;
using System;

public partial class HandCoreJoint : Node
{
	[Export] private bool _IsLeftHanded;

	[ExportCategory("References")]
	[Export] private CameraRig _CameraRig;
	[Export] private BodySolver _BodySolver;
    [Export] private RigidBody3D _HandRB;
	[Export] private RigidBody3D _CoreRB;


	[ExportCategory("Linear motion settings")]
	[Export] private float _LinearApproachTime = 0.02f;
    [Export] private float _MaxImpulsePerSecond = 10;



    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
		Transform3D wristTransform = GetWristTransform();


		HandleLinearMotion(wristTransform, delta);
    }

	private void HandleLinearMotion(Transform3D wristTransform, double delta)
	{
        //calculate hand momentum and desired hand momentum
        Vector3 handMomentum = _HandRB.LinearVelocity * _HandRB.Mass;
        Vector3 desiredHandMomentum = CalculateDesiredHandVel(wristTransform.Origin) * _HandRB.Mass;

        //calculate the required impulse to achieve this velocity, clamped to the maximum
        //we are allowed to apply per second
        Vector3 requiredImpulse = desiredHandMomentum - handMomentum;
		//ensure the hands cannot apply too much force at once by clamping the impulse
		requiredImpulse = requiredImpulse.LimitLength(_MaxImpulsePerSecond * (float)delta);

        //apply this to the hand, and apply the opposite to the core (consv. momentum)
        _HandRB.ApplyCentralImpulse(requiredImpulse);
        _CoreRB.ApplyCentralImpulse(-requiredImpulse);
    }

	private Vector3 CalculateDesiredHandVel(Vector3 wristPos)
	{
		Vector3 wristApproachVel = (wristPos - _HandRB.GlobalPosition) / _LinearApproachTime;

		return wristApproachVel + _CoreRB.LinearVelocity;
	}

	private Transform3D GetWristTransform()
	{
		if (_IsLeftHanded)
        {
            return _CameraRig.GlobalTransform * new Transform3D(_BodySolver.GetLWristBas(), _BodySolver.GetLWristPos());
        }
		else
        {
            return _CameraRig.GlobalTransform * new Transform3D(_BodySolver.GetRWristBas(), _BodySolver.GetRWristPos());
        }
    }
}
