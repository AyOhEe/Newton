using Godot;
using System;

public partial class IgnoreCollision : Node
{
	[Export] private RigidBody3D A;
	[Export] private RigidBody3D B;


	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		A.AddCollisionExceptionWith(B);
	}
}
