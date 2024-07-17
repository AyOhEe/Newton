using Godot;
using System;

public partial class PinJointController : PinJoint3D
{
	[Export] public Vector3 JointPosA;
    [Export] public Vector3 JointPosB;

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
	{
		PhysicsServer3D.PinJointSetLocalA(GetRid(), JointPosA);
        PhysicsServer3D.PinJointSetLocalB(GetRid(), JointPosB);
    }
}
