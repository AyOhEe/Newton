using Godot;
using System;

public partial class LegsJoint : Generic6DofJoint3D
{
    [ExportCategory("References")]
    [Export] private Node3D _ChestMount;
    [Export] private CameraRig _CamRig;
    [Export] private BodySolver _BodySolver;
	[Export] private CollisionShape3D _LocosphereCollision;
    [Export] private CollisionShape3D _FenderCollision;
    [Export] private Generic6DofJoint3D _FenderJoint;

    [ExportCategory("Jumping settings")]
    [Export] private float _JumpCrouchSpeed = 1;

    [ExportCategory("Crouching settings")]
    [Export] private float _CrouchSpeed = 3;
    [Export] private float _CrouchingDeadzone = 0.1f;
    [Export] private float _MinCrouchEquilibrium = 0.2f;
    [Export] private float _MaxCrouchEquilibrium = 0.9f;


    private float _WorkingCrouchSpeed;
    private float _CurrentEquilibrium;
    private bool _PreparingJump;
    private bool _ReleasedJump;


    private Vector2 _CrouchThumbstickInput;
    private float _FenderJointOffset;


    public override void _Ready()
    {
        _WorkingCrouchSpeed = _CrouchSpeed;
        _FenderJointOffset = (_FenderCollision.GlobalPosition - _FenderJoint.GlobalPosition).Y;
    }


    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
        float targetEquilibrium = RequiredLegExtension();
        //if the player just stopped preparing to jump, spring the legs back to full length
        if (_ReleasedJump)
        {
            _CurrentEquilibrium = targetEquilibrium;
            _ReleasedJump = false;
        }
        //if the player wants to jump, override the target equilibrium
        else if (_PreparingJump)
        {
            targetEquilibrium = _MinCrouchEquilibrium;
        }
        

        //if the player wants to crouch, override the target equilibrium
        if (PlayerCrouchInputActive(targetEquilibrium, out float desiredEquilibrium))
        {
            targetEquilibrium = desiredEquilibrium;
        }


        //approach the target equilibrium over time
        _CurrentEquilibrium = (float)Mathf.MoveToward(_CurrentEquilibrium, targetEquilibrium, delta * _WorkingCrouchSpeed);
        SetParamY(Param.LinearSpringEquilibriumPoint, _CurrentEquilibrium);
    }

    //TODO this needs to account for the fender
    //calculates the required extension for the leg joint such that the player camera and floor are correct
	private float RequiredLegExtension()
	{
        float chestHeight = _CamRig.GlobalBasis.Scale.Y * _BodySolver.GetChestPos().Y;
        float locosphereRadius = ((SphereShape3D)_LocosphereCollision.Shape).Radius * _LocosphereCollision.GlobalBasis.Scale.Y;
        float chestMountOffset = _ChestMount.GlobalPosition.Y - ((Node3D)_ChestMount.GetParent()).GlobalPosition.Y;

        //distance between locosphere center and chest center
        float requiredExtension = chestHeight - locosphereRadius - chestMountOffset - _FenderJointOffset;
        return requiredExtension;
    }

    private bool PlayerCrouchInputActive(float targetEquilibrium, out float desiredEquilibrium)
    {
        float absInput = Mathf.Abs(_CrouchThumbstickInput.Y);
        if (absInput <= _CrouchingDeadzone)
        {
            desiredEquilibrium = 0;
            return false;
        }

        //https://www.desmos.com/calculator/qxdvl74fdm
        float lerp = (absInput - _CrouchingDeadzone) / (1 - _CrouchingDeadzone);
        if (_CrouchThumbstickInput.Y < 0)
        {
            desiredEquilibrium = Mathf.Lerp(targetEquilibrium, _MinCrouchEquilibrium, lerp);
            desiredEquilibrium = Mathf.Min(targetEquilibrium, desiredEquilibrium);
        }
        else
        {
            desiredEquilibrium = Mathf.Lerp(targetEquilibrium, _MaxCrouchEquilibrium, lerp);
            desiredEquilibrium = Mathf.Max(targetEquilibrium, desiredEquilibrium);
        }
        return true;
    }

    public void ControllerButtonPressed(string name)
    {
        if (name == "ax_button")
        {
            _WorkingCrouchSpeed = _JumpCrouchSpeed;
            SetJumpingStatus(true);
            _ReleasedJump = false;
        }
    }
    public void ControllerButtonReleased(string name)
    {
        if (name == "ax_button")
        {
            _WorkingCrouchSpeed = _CrouchSpeed;
            SetJumpingStatus(false);
            _ReleasedJump = true;
        }
    }

    public void ControllerVec2Changed(string name, Vector2 value)
    {
        if (name == "primary")
        {
            _CrouchThumbstickInput = value;
        }
    }

    public void SetJumpingStatus(bool value)
    {
        _PreparingJump = value;
    }
}
