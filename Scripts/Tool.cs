using Godot;
using System;

public partial class Tool : Node
{
    [Export] private float _ActivationThreshold;

    [Signal] public delegate void PrimaryStartUseEventHandler(Tool tool);
    [Signal] public delegate void PrimaryUseEventHandler(Tool tool, float strength);
    [Signal] public delegate void PrimaryEndUseEventHandler(Tool tool);
    [Signal] public delegate void SecondaryStartUseEventHandler(Tool tool);
    [Signal] public delegate void SecondaryUseEventHandler(Tool tool, float strength);
    [Signal] public delegate void SecondaryEndUseEventHandler(Tool tool);

    private float _PrimaryState;
    private float _LastPrimaryState;
    private float _SecondaryState;
    private float _LastSecondaryState;

    public override void _Process(double delta)
    {
        if (_LastPrimaryState < _ActivationThreshold && _PrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryStartUse, this);
        }
        if (_PrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryUse, this, _PrimaryState);
        }
        if (_PrimaryState < _ActivationThreshold && _LastPrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryEndUse, this);
        }

        if (_LastSecondaryState < _ActivationThreshold && _SecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryStartUse, this);
        }
        if (_SecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryUse, this, _SecondaryState);
        }
        if (_SecondaryState < _ActivationThreshold && _LastSecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryEndUse, this);
        }

        _LastPrimaryState = _PrimaryState;
        _LastSecondaryState = _SecondaryState;
    }

    public void SetPrimary(float state)
    {
        _PrimaryState = state;
	}
    public void SetSecondary(float state)
    {
        _SecondaryState = state;
    }
}
