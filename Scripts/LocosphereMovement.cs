using Godot;
using System;

public partial class LocosphereMovement : RigidBody3D
{
	[Export] private BodySolver _BodySolver;
	[Export] private float _DeadzoneRadius;
	[Export] private float _AngVelMultiplier;

	Vector2 _JoystickInput = Vector2.Zero;

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
		Vector3 XZJoystickInput = _BodySolver.GetBodyDirection() * new Vector3(_JoystickInput.X, 0, -_JoystickInput.Y);
		AngularVelocity = Vector3.Up.Cross(XZJoystickInput) * _AngVelMultiplier;
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
}
