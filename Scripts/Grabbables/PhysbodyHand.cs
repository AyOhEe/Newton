using Godot;
using System;

public partial class PhysbodyHand : RigidBody3D
{
    [Export] public bool IsLeftHanded { get; private set; }
    [Export] public Node3D PalmGrabPoint { get; private set; }
}
