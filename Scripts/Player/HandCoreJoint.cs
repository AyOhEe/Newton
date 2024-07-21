using Godot;
using System;
using System.Reflection.Metadata.Ecma335;

public partial class HandCoreJoint : Node
{
	[Export] private bool _IsLeftHanded;

	[ExportCategory("References")]
	[Export] private CameraRig _CameraRig;
	[Export] private BodySolver _BodySolver;
	[Export] private GrabCoordinator _GrabCoordinator;
    [Export] private RigidBody3D _HandRB;
	[Export] private RigidBody3D _CoreRB;


    [ExportCategory("Linear motion settings")]
	[Export] private float _LinearApproachTime = 0.02f;
    [Export] private float _MaxImpulsePerSecond = 10;


	[ExportCategory("Angular motion settings")]
	[Export] private float _AngularApproachTime = 0.05f;
	[Export] private float _MaxTorque = 1;



    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
		Transform3D wristTransform = GetWristTransform();


		HandleLinearMotion(wristTransform, delta);
		HandleAngularMotion(wristTransform, delta);
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
        _HandRB.LinearVelocity += requiredImpulse / _HandRB.Mass;
        _CoreRB.LinearVelocity -= requiredImpulse / _CoreRB.Mass;
    }

	private Vector3 CalculateDesiredHandVel(Vector3 wristPos)
	{
		Vector3 clampedWristPos = ClampWristToElbow(wristPos);
		Vector3 wristApproachVel = (clampedWristPos - _HandRB.GlobalPosition) / _LinearApproachTime;

		return wristApproachVel + _CoreRB.LinearVelocity;
	}

	private Vector3 ClampWristToElbow(Vector3 wristPos)
	{
		Vector3 elbowPos = GetElbowTransform().Origin;

		return (wristPos - elbowPos).LimitLength(VRUserMeasurements.Forearm) + elbowPos;
	}


	private void HandleAngularMotion(Transform3D wristTransform, double delta)
	{
		float fdelta = (float)delta;

		//convert the current and desired rotations into quaternsions, then get the difference
		Quaternion currentRot = _HandRB.GlobalBasis.GetRotationQuaternion();
		Quaternion desiredRot = wristTransform.Basis.GetRotationQuaternion();
		Quaternion deltaQ = desiredRot * currentRot.Inverse();

		//use the smallest rotation possible (one quaternion can represent two possible rotations)
		if (deltaQ.GetAngle() > Mathf.Pi)
		{
			deltaQ = -deltaQ;
		}

		//use this to calculate the axis and angle of rotation
		Vector3 desiredAngVel = (deltaQ.GetAxis() * deltaQ.GetAngle()) / _AngularApproachTime;
		Vector3 desiredAngMomentum = _GrabCoordinator.CalculateDesiredAngularMomentum(_HandRB, desiredAngVel, _IsLeftHanded);
		Vector3 currentAngMomentum = _HandRB.GetInverseInertiaTensor().Inverse() * _HandRB.AngularVelocity;
		desiredAngMomentum = (desiredAngMomentum - currentAngMomentum).LimitLength(fdelta * _MaxTorque) + currentAngMomentum;
        _HandRB.AngularVelocity = _HandRB.GetInverseInertiaTensor() * desiredAngMomentum;
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
    private Transform3D GetElbowTransform()
    {
        if (_IsLeftHanded)
        {
            return _CameraRig.GlobalTransform * new Transform3D(_BodySolver.GetLElbowBas(), _BodySolver.GetLElbowPos());
        }
        else
        {
            return _CameraRig.GlobalTransform * new Transform3D(_BodySolver.GetRElbowBas(), _BodySolver.GetRElbowPos());
        }
    }
}
