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
        //we need to calculate the local inertia tensor before rotating any rigidbody
        Basis leftHandIT = PhysicsHelpers.GetLocalInertiaTensor(_LeftHandRB);
        Basis rightHandIT = PhysicsHelpers.GetLocalInertiaTensor(_RightHandRB);
        Basis grabbableIT = PhysicsHelpers.GetLocalInertiaTensor(_GrabbableRB);


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


        //TODO basis should be set proportional to the inertia of the objects in question
        Quaternion lRot = (_LeftHandRB.GlobalBasis * _LeftTargetRotation).GetRotationQuaternion();
        Quaternion rRot = (_RightHandRB.GlobalBasis * _RightTargetRotation).GetRotationQuaternion();
        Quaternion lerpedRot = lRot.Slerp(rRot, 0.5f);
        _GrabbableRB.GlobalBasis = new Basis(lerpedRot);
        _LeftHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _LeftTargetRotation.Inverse();
        _RightHandRB.GlobalBasis = _GrabbableRB.GlobalBasis * _RightTargetRotation.Inverse();


        //now that we've applied our rotations, recalculate the inertia tensors
        leftHandIT = PhysicsHelpers.RotateInertiaTensor(leftHandIT, _LeftHandRB.GlobalBasis);
        rightHandIT = PhysicsHelpers.RotateInertiaTensor(rightHandIT, _RightHandRB.GlobalBasis);
        grabbableIT = PhysicsHelpers.RotateInertiaTensor(grabbableIT, _GrabbableRB.GlobalBasis);


        //we'll need these for the upcoming calculations
        Vector3 lHandCOM = _LeftHandRB.GlobalTransform * _LeftHandRB.CenterOfMass;
        Vector3 rHandCOM = _RightHandRB.GlobalTransform * _RightHandRB.CenterOfMass;
        Vector3 grabbableCOM = _GrabbableRB.GlobalTransform * _GrabbableRB.CenterOfMass;
        Vector3 COM = PhysicsHelpers.CalculateCentreOfMass(_LeftHandRB, _RightHandRB, _GrabbableRB);
        Basis totalInertia = PhysicsHelpers.CombineInertiaTensors(
            COM,
            new RigidBody3D[] { _LeftHandRB, _RightHandRB, _GrabbableRB },
            new Basis[] { leftHandIT, rightHandIT, grabbableIT }
        );

        //the momentum of the COM will be the combined linear momentum of the three bodies. from there, velocity
        Vector3 COMMomentum = (_LeftHandRB.LinearVelocity * _LeftHandRB.Mass) 
            + (_RightHandRB.LinearVelocity * _RightHandRB.Mass)
            + (_GrabbableRB.LinearVelocity * _GrabbableRB.Mass);
        Vector3 COMVel = COMMomentum / (_LeftHandRB.Mass + _RightHandRB.Mass + _GrabbableRB.Mass);


        //https://www.physicsforums.com/threads/total-angular-momentum-of-2-connected-falling-bodies.566219/
        //not quite the same thing, but extending the principle
        Vector3 angularMomentum = (leftHandIT * _LeftHandRB.AngularVelocity)
            + (rightHandIT * _RightHandRB.AngularVelocity)
            + (grabbableIT * _GrabbableRB.AngularVelocity)
            + (_LeftHandRB.Mass * (lHandCOM - COM).Cross(_LeftHandRB.LinearVelocity - COMVel))
            + (_RightHandRB.Mass * (rHandCOM - COM).Cross(_RightHandRB.LinearVelocity - COMVel))
            + (_GrabbableRB.Mass * (grabbableCOM - COM).Cross(_GrabbableRB.LinearVelocity - COMVel));
        Vector3 angularVelocity = totalInertia.Inverse() * angularMomentum;

        //the bodies will have the same angular velocity as they are joined together perfectly
        _GrabbableRB.AngularVelocity = angularVelocity;
        _LeftHandRB.AngularVelocity = angularVelocity;
        _RightHandRB.AngularVelocity = angularVelocity;


        //from COM velocity and angular velocity, we can calculate the linear velocity
        _LeftHandRB.LinearVelocity = (COMVel + (lHandCOM - COM).Cross(angularVelocity));
        _RightHandRB.LinearVelocity = (COMVel + (rHandCOM - COM).Cross(angularVelocity));
        _GrabbableRB.LinearVelocity = (COMVel + (grabbableCOM - COM).Cross(angularVelocity));
    }

    public static Basis AddBases(Basis left, Basis right)
    {
        return new Basis(left.Column0 + right.Column0, left.Column1 + right.Column1, left.Column2 + right.Column2);
    }
}
