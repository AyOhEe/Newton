using Godot;
using System;

public partial class LocosphereMovement : RigidBody3D
{

	[ExportCategory("References")]
	[Export] private CameraRig _CameraRig;
	[Export] private BodySolver _BodySolver;
	[Export] private float _DeadzoneRadius;

	[ExportCategory("Speed settings")]
    [Export] private float _CrouchedAngVelMultiplier = 0.25f;
    [Export] private float _DefaultAngVel = 8;
	[Export] private float _SprintingAngVelMultiplier = 1.25f;


	Vector2 _JoystickInput = Vector2.Zero;
	float _AngVelMultiplier = 1;
	bool IsSprinting = false;


	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
		_AngVelMultiplier = 1;
		if (IsCrouching())
		{
			_AngVelMultiplier *= _CrouchedAngVelMultiplier;
		}
		if (IsSprinting)
		{
			_AngVelMultiplier *= _SprintingAngVelMultiplier;
		}
		float angVel = _DefaultAngVel * _AngVelMultiplier;


        Basis bodyDir = _CameraRig.GlobalBasis * _BodySolver.GetBodyDirection();
        Vector3 XZJoystickInput = bodyDir * new Vector3(_JoystickInput.X, 0, -_JoystickInput.Y);
		AngularVelocity = Vector3.Up.Cross(XZJoystickInput).Normalized() * angVel;
	}

	//TODO this sucks, but I can't do better until the measurements system is in place
	//this should really also take crouching input into account
	private bool IsCrouching()
	{
		return _CameraRig.Camera.Position.Y < 1f;
	}

	public void ControllerVec2Input(string name, Vector2 value)
	{
		if (name == "primary")
		{
			if (value.Length() < _DeadzoneRadius)
			{
				_JoystickInput = Vector2.Zero;
				return;
			}
			_JoystickInput = value;
		}
	}

	public void ControllerButtonInput(string name)
	{
		if (name == "primary_click")
		{
			IsSprinting = !IsSprinting;
		}
	}
}
