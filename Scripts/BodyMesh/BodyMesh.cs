using Godot;
using Godot.Collections;
using System;

public partial class BodyMesh : Node
{
    [Export] private Array<Node3D> _BoneAttachments;
    [Export] private Array<Node3D> _BoneAttachmentMounts;

    public override void _EnterTree()
    {
        if(_BoneAttachmentMounts.Count != _BoneAttachments.Count)
        {
            GD.PrintErr("BodyMesh was not provided with equal bone attachments and bone group mounts!");
        }
    }

    public override void _Process(double delta)
    {
        if (_BoneAttachmentMounts.Count != _BoneAttachments.Count)
        {
            return;
        }

        for(int i = 0; i < _BoneAttachments.Count; i++)
        {
            if(_BoneAttachments[i] == null || _BoneAttachmentMounts[i] == null)
            {
                continue;
            }

            _BoneAttachments[i].GlobalTransform = _BoneAttachmentMounts[i].GlobalTransform;
        }
    }
}
