using Godot;
using System;

public partial class Tool : Node
{
    [Export] private float _ActivationThreshold;

    [Signal] public delegate void PrimaryStartUseEventHandler();
    [Signal] public delegate void PrimaryUseEventHandler(float strength);
    [Signal] public delegate void PrimaryEndUseEventHandler();
    [Signal] public delegate void SecondaryStartUseEventHandler();
    [Signal] public delegate void SecondaryUseEventHandler(float strength);
    [Signal] public delegate void SecondaryEndUseEventHandler();

    private float _PrimaryState;
    private float _LastPrimaryState;
    private float _SecondaryState;
    private float _LastSecondaryState;

    public override void _Process(double delta)
    {
        if (_LastPrimaryState < _ActivationThreshold && _PrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryStartUse);
        }
        if (_PrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryUse, _PrimaryState);
        }
        if (_PrimaryState < _ActivationThreshold && _LastPrimaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.PrimaryEndUse);
        }

        if (_LastSecondaryState < _ActivationThreshold && _SecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryStartUse);
        }
        if (_SecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryUse, _SecondaryState);
        }
        if (_SecondaryState < _ActivationThreshold && _LastSecondaryState >= _ActivationThreshold)
        {
            EmitSignal(SignalName.SecondaryEndUse);
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
