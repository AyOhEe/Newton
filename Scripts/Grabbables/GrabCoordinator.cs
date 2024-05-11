using Godot;
using System;
using static Godot.OpenXRInterface;

public partial class GrabCoordinator : Node
{
	private GrabbableJoint _LeftHandJoint;
	private GrabbableJoint _RightHandJoint;
	private TwoHandedGrabbableJoint _THJoint;
	private Transform3D _LeftHandPose;
	private Transform3D _RightHandPose;

	public void CreateJoint(Grabbable target, PhysbodyHand hand, Transform3D parentspacePose)
	{
        RigidBody3D leftHandTarget = _LeftHandJoint?._GrabbableRB;
        RigidBody3D rightHandTarget = _RightHandJoint?._GrabbableRB;

        //replace the current hand's target with their current target
        if (hand.IsLeftHanded)
        {
            leftHandTarget = target.ParentRigidBody;
        }
        else
        {
            rightHandTarget = target.ParentRigidBody;
        }

        //if both hands want to grab the same thing, create a two handed joint.
        //two one handed joints would not (and DO not) behave correctly
        if(leftHandTarget == rightHandTarget)
        {
            CreateTwoHandedJoint(target, hand, parentspacePose);
        }
        else
        {
            CreateOneHandedJoint(target, hand, parentspacePose);
        }
    }

    //handles creating a joint for one handed grabbing
	private void CreateOneHandedJoint(Grabbable target, PhysbodyHand hand, Transform3D parentspacePose)
	{
        if (hand.IsLeftHanded)
        {
            if (_LeftHandJoint != null)
            {
                GD.PushError("Attempting to grab while joint exists!");
                return;
            }
            _LeftHandJoint = _CreateOneHandedJoint(target, hand, parentspacePose);
            _LeftHandPose = parentspacePose;
        }
        else
        {
            if (_RightHandJoint != null)
            {
                GD.PushError("Attempting to grab while joint exists!");
                return;
            }
            _RightHandJoint = _CreateOneHandedJoint(target, hand, parentspacePose);
            _RightHandPose = parentspacePose;
        }
    }
	//creates a joint for one handed grabbing
	private GrabbableJoint _CreateOneHandedJoint(Grabbable target, PhysbodyHand hand, Transform3D parentspacePose)
	{
		GrabbableJoint joint = new GrabbableJoint(hand, target.ParentRigidBody);

        joint.TargetRotation = hand.PalmGrabPoint.Basis * parentspacePose.Basis.Inverse();
        joint.TargetPosition = hand.PalmGrabPoint.Position + (joint.TargetRotation * -parentspacePose.Origin);

        target.ParentRigidBody.AddChild(joint);
		return joint;
    }

    //handles creating a joint for two handed grabbing
    private void CreateTwoHandedJoint(Grabbable target, PhysbodyHand hand, Transform3D parentspacePose)
    {
        //create the onehanded joint for the new hand, but remove it from the scene tree.
        //we want this to exist so we can use it later if the other hand releases first.
        //conveniently, one handed joints also handle collision for us! the two handed won't need
        //to think about that.
        if (hand.IsLeftHanded)
        {
            _LeftHandJoint = _CreateOneHandedJoint(target, hand, parentspacePose);
            _LeftHandPose = parentspacePose; 
        }
        else
        {
            _RightHandJoint = _CreateOneHandedJoint(target, hand, parentspacePose);
            _RightHandPose = parentspacePose;
        }

        //remove both joints from the scene tree so they won't get _PhysicsProcess called
        _LeftHandJoint.GetParent().RemoveChild(_LeftHandJoint);
        _RightHandJoint.GetParent().RemoveChild(_RightHandJoint);

        //now that the one handed joints won't interfere, create the two handed joint
        _THJoint = _CreateTwoHandedJoint();
    }
    //creates a joint for one handed grabbing
    private TwoHandedGrabbableJoint _CreateTwoHandedJoint()
    {
        //we can use the information in the already existing one handed joints to construct this one
        TwoHandedGrabbableJoint joint = new TwoHandedGrabbableJoint(_LeftHandJoint, _RightHandJoint);
        _LeftHandJoint._GrabbableRB.AddChild(joint);


        return joint;
    }

    public void DestroyJoint(PhysbodyHand hand)
	{
        //two handed joints
        if (_THJoint != null)
        {
            if (hand.IsLeftHanded)
            {
                //the other hand's joint should still have control
                //this is why we didn't free it when the two handed grab started!
                _THJoint.GetParent().AddChild(_RightHandJoint);

                _LeftHandJoint.HandleDestruction();
                _LeftHandJoint.QueueFree();
                _LeftHandJoint = null;
            }
            else
            {
                //the other hand's joint should still have control.
                //this is why we didn't free it when the two handed grab started!
                _THJoint.GetParent().AddChild(_LeftHandJoint);

                _RightHandJoint.HandleDestruction();
                _RightHandJoint.QueueFree();
                _RightHandJoint = null;
            }

            _THJoint.QueueFree();
            _THJoint = null;

            return;
        }

        //one handed joints
		if (hand.IsLeftHanded)
		{
			if (_LeftHandJoint == null)
			{
				return;
			}
            _LeftHandJoint.HandleDestruction();
            _LeftHandJoint.QueueFree();
            _LeftHandJoint = null;
        }
		else
		{
			if(_RightHandJoint == null)
			{
				return;
			}
            _RightHandJoint.HandleDestruction();
            _RightHandJoint.QueueFree();
            _RightHandJoint = null;
        }
	}


    public Vector3 CalculateDesiredAngularMomentum(RigidBody3D handRB, Vector3 angVel, bool leftHanded)
    {
        if (leftHanded)
        {
            if (_LeftHandJoint == null)
            {
                return handRB.GetInverseInertiaTensor().Inverse() * angVel;
            }

            //the momentum of the COM will be the combined linear momentum of the two bodies. from there, velocity
            RigidBody3D grabbable = _LeftHandJoint._GrabbableRB;
            Vector3 COMMomentum = (handRB.LinearVelocity * handRB.Mass) + (grabbable.LinearVelocity * grabbable.Mass);
            Vector3 COMVel = COMMomentum / (handRB.Mass + grabbable.Mass);

            Vector3 COM = PhysicsHelpers.CalculateCentreOfMass(handRB, grabbable);
            return (grabbable.GetInverseInertiaTensor().Inverse() * (angVel - grabbable.AngularVelocity))
                 + (handRB.GetInverseInertiaTensor().Inverse() * angVel)
                 + (handRB.Mass * (handRB.CenterOfMass - COM).Cross(handRB.LinearVelocity - COMVel))
                 + (grabbable.Mass * (grabbable.CenterOfMass - COM).Cross(grabbable.LinearVelocity - COMVel));
        }
        else
        {
            if (_RightHandJoint == null)
            {
                return handRB.GetInverseInertiaTensor().Inverse() * angVel;
            }

            //the momentum of the COM will be the combined linear momentum of the two bodies. from there, velocity
            RigidBody3D grabbable = _RightHandJoint._GrabbableRB;
            Vector3 COMMomentum = (handRB.LinearVelocity * handRB.Mass) + (grabbable.LinearVelocity * grabbable.Mass);
            Vector3 COMVel = COMMomentum / (handRB.Mass + grabbable.Mass);

            Vector3 COM = PhysicsHelpers.CalculateCentreOfMass(handRB, grabbable);
            return (grabbable.GetInverseInertiaTensor().Inverse() * (angVel - grabbable.AngularVelocity))
                 + (handRB.GetInverseInertiaTensor().Inverse() * angVel)
                 + (handRB.Mass * (handRB.CenterOfMass - COM).Cross(handRB.LinearVelocity - COMVel))
                 + (grabbable.Mass * (grabbable.CenterOfMass - COM).Cross(grabbable.LinearVelocity - COMVel));
        }
    }
}
