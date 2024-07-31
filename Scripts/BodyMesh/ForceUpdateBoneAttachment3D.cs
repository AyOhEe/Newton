using Godot;
using System;

public partial class ForceUpdateBoneAttachment3D : BoneAttachment3D
{
	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		GlobalTransform = GlobalTransform;
	}
}
