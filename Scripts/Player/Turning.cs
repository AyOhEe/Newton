using Godot;
using Godot.Collections;
using System;
using System.Collections;
using System.Collections.Generic;

public partial class Turning : Node
{
    public enum TurningMode
    {
        Disabled,
        SnapTurning,
        SmoothTurning,
    }

    [ExportCategory("References")]
    [Export] private CameraRig _CameraRig;
    [Export] private Node3D _TurnPivot;
    [Export] private Timer _SnapTurnResetTimer;

    [ExportCategory("Turning settings")]
    [Export] public TurningMode _TurnMode = TurningMode.SnapTurning;
    [Export] private float _DeadzoneRadius = 0.1f;
    [Export] private float _SmoothTurnPerSecond = 175;

    [ExportCategory("Snap turn settings")]
    [Export] private Array<float> _SnapTurnFrames = new Array<float> { 9f, 9f, 9f, 9f, 9f };
    [Export] private float _SnapTurnInputThreshold = 0.8f;
    [Export] private float _SnapTurnResetDelay = 0.15f;


    Vector2 _ThumbstickInput = Vector2.Zero;


    IEnumerator _SnapTurnCoroutine = null;
    private bool _CanSnapTurn = true;


    public override void _PhysicsProcess(double delta)
	{
        switch (_TurnMode)
        {
            case TurningMode.Disabled:
                _SnapTurnCoroutine = null;
                break;
            case TurningMode.SnapTurning:
                HandleSnapTurning();
                break;
            case TurningMode.SmoothTurning:
                HandleSmoothTurning((float)delta);
                break;
        }
	}

    private void HandleSnapTurning()
    {
        //check to make sure we're not already turning
        if (_SnapTurnCoroutine != null)
        {
            //in the middle of a turn. don't trigger another, just process this one
            if (!_SnapTurnCoroutine.MoveNext())
            {
                //turn has ended, get rid of the ienumerator and stop
                _SnapTurnCoroutine = null;
            }
            return;
        }
        //and check to make sure we're allowed to start a new turn
        if (!_CanSnapTurn)
        {
            //we're not allowed to turn right now. 
            return;
        }

        //get the player's turning input
        float turningInput = CalculateTurningInput();
        if (Mathf.Abs(turningInput) < _SnapTurnInputThreshold)
        {
            //not enough input to trigger a turn
            return;
        }


        if (turningInput < 0)
        {
            //left turn
            _SnapTurnCoroutine = SnapTurn(_CameraRig, _TurnPivot, Vector3.Up, _SnapTurnFrames);
        }
        else
        {
            //right turn
            //turning about the negative of an axis flips the direction of rotation
            _SnapTurnCoroutine = SnapTurn(_CameraRig, _TurnPivot, Vector3.Down, _SnapTurnFrames);
        }
    }

    private IEnumerator SnapTurn(Node3D target, Node3D pivot, Vector3 axis, IEnumerable<float> turnAmounts)
    {
        _CanSnapTurn = false;
        foreach(float theta in turnAmounts)
        {
            TurnAboutPivot(target, pivot.GlobalPosition, axis, theta);
            yield return null;
        }
        _SnapTurnResetTimer.Start(_SnapTurnResetDelay);
    }

    private void HandleSmoothTurning(float f_delta)
    {
        float turningInput = CalculateTurningInput();
        float turnDirection = Mathf.Sign(turningInput);
        if (turnDirection == 0)
        {
            //no input: do nothing
            return;
        }

        float theta = f_delta * turningInput * _SmoothTurnPerSecond;
        TurnAboutPivot(_CameraRig, _TurnPivot.GlobalPosition, Vector3.Up, -theta);
    }

    private float CalculateTurningInput()
    {
        float absInput = Mathf.Abs(_ThumbstickInput.X);
        if (absInput <= _DeadzoneRadius)
        {
            return 0;
        }

        //https://www.desmos.com/calculator/a47gadzzep
        return (absInput - _DeadzoneRadius) / (1 - _DeadzoneRadius) * Mathf.Sign(_ThumbstickInput.X);
    }

    private void TurnAboutPivot(Node3D target, Vector3 pivot, Vector3 axis, float theta)
    {
        Basis rotation = new Basis(axis, Mathf.DegToRad(theta));

        target.GlobalBasis *= rotation;
        target.GlobalPosition = pivot + (rotation * (target.GlobalPosition - pivot));
    }

    public void ControllerVec2Input(string name, Vector2 value)
    {
        if (name == "primary")
        {
            _ThumbstickInput = value;
        }
    }

    public void ReEnableSnapTurn()
    {
        _CanSnapTurn = true;
    }
}
