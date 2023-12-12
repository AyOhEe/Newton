using Godot;
using System;

public partial class LegsJoint : Generic6DofJoint3D
{
    [ExportCategory("References")]
    [Export] private Node3D _ChestMount;
    [Export] private CameraRig _CamRig;
    [Export] private BodySolver _BodySolver;
	[Export] private CollisionShape3D _LocosphereCollision;

    [ExportCategory("Jumping settings")]
    [Export] private float _CrouchingEquilibrium;
    [Export] private float _CrouchSpeed = 3;
    [Export] private float _JumpCrouchSpeed = 1;


    private float _WorkingCrouchSpeed;
    private float _CurrentEquilibrium;
    private bool _PreparingJump;
    private bool _ReleasedJump;


    public override void _Ready()
    {
        _WorkingCrouchSpeed = _CrouchSpeed;
    }


    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _PhysicsProcess(double delta)
	{
        float targetEquilibrium = RequiredLegExtension();
        if (_PreparingJump)
        {
            targetEquilibrium = _CrouchingEquilibrium;
        }
        else if (_ReleasedJump)
        {
            _CurrentEquilibrium = targetEquilibrium;
            _ReleasedJump = false;
        }

        _CurrentEquilibrium = (float)Mathf.MoveToward(_CurrentEquilibrium, targetEquilibrium, delta * _WorkingCrouchSpeed);
        SetParamY(Param.LinearSpringEquilibriumPoint, _CurrentEquilibrium);
    }

    //calculates the required extension for the leg joint such that the player camera and floor are correct
	private float RequiredLegExtension()
	{
        float chestHeight = _CamRig.GlobalBasis.Scale.Y * _BodySolver.GetChestPos().Y;
        float locosphereRadius = ((SphereShape3D)_LocosphereCollision.Shape).Radius * _LocosphereCollision.GlobalBasis.Scale.Y;
        float chestMountOffset = _ChestMount.GlobalPosition.Y - ((Node3D)_ChestMount.GetParent()).GlobalPosition.Y;

        //distance between locosphere center and chest center
        float requiredExtension = chestHeight - locosphereRadius - chestMountOffset;
        return requiredExtension;
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

    public void SetJumpingStatus(bool value)
    {
        _PreparingJump = value;
    }
}
