using Godot;
using System;

public partial class HeadCollider : Node3D
{
	[Export] private CameraRig _CameraRig;
    [Export] private BodySolver _BodySolver;
    [Export] private Node3D _EyeMount;
    [Export] private Node3D _ChestMount;


    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
	{
		Transform = _ChestMount.Transform
			* new Transform3D(_BodySolver.GetChestBas(), _BodySolver.GetChestPos()).Inverse()
			* new Transform3D(_BodySolver.GetEyesBas(), _BodySolver.GetEyesPos())
			* _EyeMount.Transform.Inverse();
	}
}
