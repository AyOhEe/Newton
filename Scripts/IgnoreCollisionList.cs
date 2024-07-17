using Godot;
using Godot.Collections;
using System;

public partial class IgnoreCollisionList : Node
{
    [Export] private Array<RigidBody3D> BodiesA;
    [Export] private Array<RigidBody3D> BodiesB;


    public override void _EnterTree()
    {
        if (BodiesA.Count != BodiesB.Count)
        {
            GD.PrintErr("IgnoreCollisionList provided with unequal lists of RigidBody3Ds");
            return;
        }

        for (int i = 0; i < BodiesA.Count; i++)
        {
            BodiesA[i].AddCollisionExceptionWith(BodiesB[i]);
        }
    }
    public override void _ExitTree()
    {
        if (BodiesA.Count != BodiesB.Count)
        {
            GD.PrintErr("IgnoreCollisionList provided with unequal lists of RigidBody3Ds");
            return;
        }

        for (int i = 0; i < BodiesA.Count; i++)
        {
            BodiesA[i].RemoveCollisionExceptionWith(BodiesB[i]);
        }
    }
}
