using Godot;
using Godot.Collections;
using System;

public partial class ShrinkMidair : Node
{
    [Export] private ShapeCast3D _GroundCheck;
    [Export] private Array<Node3D> ignoreList;
    [Export] private CollisionShape3D _CollisionShape;
    [Export] private float normalRadius;
    [Export] private float shrankRadius;
    [Export] private float tweenTime;

    private Tween _RadiusTweener;

    public override void _EnterTree()
    {
        foreach(CollisionObject3D co in ignoreList)
        {
            _GroundCheck.AddException(co);
        }
    }
    public override void _ExitTree()
    {
        foreach (CollisionObject3D co in ignoreList)
        {
            _GroundCheck.RemoveException(co);
        }
    }

    private void UpdateCollider()
    {
        if (_GroundCheck.IsColliding())
        {
            Expand();
        }
        else
        {
            Shrink();
        }
    }

    private void Expand()
    {
        _RadiusTweener?.Kill();
        _RadiusTweener = CreateTween();
        _RadiusTweener.TweenProperty(_CollisionShape.Shape, "radius", normalRadius, tweenTime).FromCurrent();
    }
    private void Shrink()
    {
        _RadiusTweener?.Kill();
        _RadiusTweener = CreateTween();
        _RadiusTweener.TweenProperty(_CollisionShape.Shape, "radius", shrankRadius, tweenTime).FromCurrent();
    }
}
