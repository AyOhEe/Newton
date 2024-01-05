using Godot;
using System;

public partial class TwoHandedGrabbableJoint : Node
{
    private PhysbodyHand _LeftHandRB;
    private PhysbodyHand _RightHandRB;
    private RigidBody3D _GrabbableRB;

    private Vector3 _LeftTargetPosition;
    private Basis _LeftTargetRotation;
    private Vector3 _RightTargetPosition;
    private Basis _RightTargetRotation;

    public TwoHandedGrabbableJoint(GrabbableJoint leftJoint, GrabbableJoint rightJoint)
    {
        _LeftHandRB = leftJoint._HandRB;
        _RightHandRB = rightJoint._HandRB;
        _GrabbableRB = rightJoint._GrabbableRB;

        _LeftTargetPosition = leftJoint.TargetPosition;
        _LeftTargetRotation = leftJoint.TargetRotation;
        _RightTargetPosition = rightJoint.TargetPosition;
        _RightTargetRotation = rightJoint.TargetRotation;
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_GrabbableRB.Freeze)
        {
            HandleFrozenGrab(delta);
        }
        else
        {
            HandleNonFrozenGrab(delta);
        }
    }
    private void HandleFrozenGrab(double delta)
    {
        //the hand is essentially stuck to the frozen body right now. as such, move the hand to the required
        //position and rotation
        _LeftHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _LeftTargetRotation.Inverse();
        _LeftHandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_LeftHandRB.GlobalBasis * _LeftTargetPosition);
        _LeftHandRB.LinearVelocity = Vector3.Zero; //if we don't do this, the hand doesn't know it's being held back
        _LeftHandRB.AngularVelocity = Vector3.Zero;

        _RightHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _RightTargetRotation.Inverse();
        _RightHandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_RightHandRB.GlobalBasis * _RightTargetPosition);
        _RightHandRB.LinearVelocity = Vector3.Zero; //if we don't do this, the hand doesn't know it's being held back
        _RightHandRB.AngularVelocity = Vector3.Zero;
    }


    private void HandleNonFrozenGrab(double delta)
    {
        //TODO angular momentum stuff


        //TODO remove this once the angular momentum calculations work
        Quaternion lRot = (_LeftHandRB.GlobalBasis * _LeftTargetRotation).GetRotationQuaternion();
        Quaternion rRot = (_RightHandRB.GlobalBasis * _RightTargetRotation).GetRotationQuaternion();
        Quaternion lerpedRot = lRot.Slerp(rRot, 0.5f);
        _GrabbableRB.GlobalBasis = new Basis(lerpedRot);
        _LeftHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _LeftTargetRotation.Inverse();
        _RightHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _RightTargetRotation.Inverse();



        //the bodies are effectively fused right now. as such, position them together, moving them
        //inversely proportional to their masses
        Vector3 lGrabbableDesiredPos = _LeftHandRB.GlobalPosition + (_LeftHandRB.GlobalBasis * _LeftTargetPosition);
        Vector3 rGrabbableDesiredPos = _RightHandRB.GlobalPosition + (_RightHandRB.GlobalBasis * _RightTargetPosition);
        float lWeighting = _LeftHandRB.Mass / (_LeftHandRB.Mass + _RightHandRB.Mass + _GrabbableRB.Mass);
        float rWeighting = _RightHandRB.Mass / (_LeftHandRB.Mass + _RightHandRB.Mass + _GrabbableRB.Mass);
        Vector3 lTargetPos = _GrabbableRB.GlobalPosition.Lerp(lGrabbableDesiredPos, lWeighting);
        Vector3 rTargetPos = _GrabbableRB.GlobalPosition.Lerp(rGrabbableDesiredPos, rWeighting);

        float rHandWeight = _RightHandRB.Mass / (_LeftHandRB.Mass + _RightHandRB.Mass);
        Vector3 targetPos = lTargetPos.Lerp(rTargetPos, rHandWeight);

        _GrabbableRB.MoveAndCollide(targetPos - _GrabbableRB.GlobalPosition);
        _LeftHandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_LeftHandRB.GlobalBasis * _LeftTargetPosition);
        _RightHandRB.GlobalPosition = _GrabbableRB.GlobalPosition - (_RightHandRB.GlobalBasis * _RightTargetPosition);


        //their velocities must also be equal. average them based on total momentum
        Vector3 totalMomentum = (_LeftHandRB.LinearVelocity * _LeftHandRB.Mass)
            + (_RightHandRB.LinearVelocity * _RightHandRB.Mass)
            + (_GrabbableRB.LinearVelocity * _GrabbableRB.Mass);
        Vector3 targetVel = totalMomentum / (_LeftHandRB.Mass + _RightHandRB.Mass + _GrabbableRB.Mass);
        _LeftHandRB.LinearVelocity = targetVel;
        _RightHandRB.LinearVelocity = targetVel;
        _GrabbableRB.LinearVelocity = targetVel;
    }
}
