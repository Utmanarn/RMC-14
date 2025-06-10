using System.Linq;
using Content.Shared._RMC14.Weapons.Ranged.IFF;
using Content.Shared.Interaction;
using Content.Shared.Physics;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Prototypes;
using static Content.Shared.Interaction.SharedInteractionSystem;

namespace Content.Shared._RMC14.LineOfSight;

public sealed partial class LineOfSightSystem : EntitySystem
{
    [Dependency] private readonly SharedInteractionSystem _interactionSystem = default!;
    [Dependency] private readonly SharedPhysicsSystem _broadphase = default!;
    [Dependency] private readonly GunIFFSystem _iffSystem = default!;


    /// <summary>
    ///     Checks that these coordinates are within a certain distance without any
    ///     entity that matches the collision mask obstructing them.
    ///     If the <paramref name="range"/> is zero or negative,
    ///     this method will only check if nothing obstructs the two sets
    ///     of coordinates.
    /// </summary>
    /// <param name="origin">Set of coordinates to use.</param>
    /// <param name="other">Other set of coordinates to use.</param>
    /// <param name="range">
    ///     Maximum distance between the two sets of coordinates.
    /// </param>
    /// <param name="collisionMask">The mask to check for collisions.</param>
    /// <param name="predicate">
    ///     A predicate to check whether to ignore an entity or not.
    ///     If it returns true, it will be ignored.
    /// </param>
    /// <param name="checkAccess">Perform range checks</param>
    /// <returns>
    ///     True if the two points are within a given range without being obstructed.
    /// </returns>
    public bool InRangeUnobstructed(
        MapCoordinates origin,
        MapCoordinates other,
        float range = InteractionRange,
        CollisionGroup collisionMask = InRangeUnobstructedMask,
        Ignored? predicate = null,
        bool checkAccess = true,
        EntProtoId<IFFFactionComponent>? faction = null)
    {
        // Have to be on same map regardless.
        if (other.MapId != origin.MapId)
            return false;

        if (!checkAccess)
            return true;

        var dir = other.Position - origin.Position;
        var length = dir.Length();

        // If range specified also check it
        if (range > 0f && length > range)
            return false;

        if (MathHelper.CloseTo(length, 0))
            return true;

        predicate ??= _ => false;

        if (length > MaxRaycastRange)
        {
            Log.Warning("InRangeUnobstructed check performed over extreme range. Limiting CollisionRay size.");
            length = MaxRaycastRange;
        }

        var ray = new CollisionRay(origin.Position, dir.Normalized(), (int)collisionMask);
        var rayResults = _broadphase.IntersectRayWithPredicate(origin.MapId, ray, length, predicate.Invoke, false).ToList();

        if (faction != null)
            // Check IFF then remove the result from the list in case they are in the same faction.
            foreach (var result in rayResults)
            {
                if (!_iffSystem.IsInFaction(result.HitEntity, faction.Value))
                    break;
                else
                {
                    rayResults.Remove(result); // Maybe don't remove rays from a dictionary that is being enumerated over.
                }
            }

        return rayResults.Count == 0;
    }

    /// <summary>
    ///     Checks that two entities are within a certain distance without any
    ///     entity that matches the collision mask obstructing them.
    ///     If the <paramref name="range"/> is zero or negative,
    ///     this method will only check if nothing obstructs the two entities.
    ///     This function will also check whether the other entity is a wall-mounted entity. If it is, it will
    ///     automatically ignore some obstructions.
    /// </summary>
    /// <param name="origin">The first entity to use.</param>
    /// <param name="other">Other entity to use.</param>
    /// <param name="otherAngle">The local rotation to use for the other entity.</param>
    /// <param name="range">
    ///     Maximum distance between the two entities.
    /// </param>
    /// <param name="collisionMask">The mask to check for collisions.</param>
    /// <param name="predicate">
    ///     A predicate to check whether to ignore an entity or not.
    ///     If it returns true, it will be ignored.
    /// </param>
    /// <param name="popup">
    ///     Whether or not to popup a feedback message on the origin entity for
    ///     it to see.
    /// </param>
    /// <param name="otherCoordinates">The coordinates to use for the other entity.</param>
    /// <returns>
    ///     True if the two points are within a given range without being obstructed.
    /// </returns>
    /// <param name="overlapCheck">If true, if the broadphase query returns an overlap (0f distance) this function will early out true with no raycast made.</param>
    public bool InRangeUnobstructed(
        Entity<TransformComponent?> origin,
        Entity<TransformComponent?> other,
        EntityCoordinates otherCoordinates,
        Angle otherAngle,
        float range = InteractionRange,
        CollisionGroup collisionMask = InRangeUnobstructedMask,
        Ignored? predicate = null,
        bool popup = false,
        bool overlapCheck = true)
    {
        Ignored combinedPredicate = e => e == origin.Owner || (predicate?.Invoke(e) ?? false);
        var inRange = true;
        MapCoordinates originPos = default;
        var targetPos = _transform.ToMapCoordinates(otherCoordinates);
        Angle targetRot = default;

        // So essentially:
        // 1. If fixtures available check nearest point. We take in coordinates / angles because we might want to use a lag compensated position
        // 2. Fall back to centre of body.

        // Alternatively we could check centre distances first though
        // that means we wouldn't be able to easily check overlap interactions.
        if (range > 0f &&
            _fixtureQuery.TryComp(origin, out var fixtureA) &&
            // These fixture counts are stuff that has the component but no fixtures for <reasons> (e.g. buttons).
            // At least until they get removed.
            fixtureA.FixtureCount > 0 &&
            _fixtureQuery.TryComp(other, out var fixtureB) &&
            fixtureB.FixtureCount > 0 &&
            Resolve(origin, ref origin.Comp))
        {
            var (worldPosA, worldRotA) = _transform.GetWorldPositionRotation(origin.Comp);
            var xfA = new Transform(worldPosA, worldRotA);
            var parentRotB = _transform.GetWorldRotation(otherCoordinates.EntityId);
            var xfB = new Transform(targetPos.Position, parentRotB + otherAngle);

            // Different map or the likes.
            if (!_broadphase.TryGetNearest(
                    origin,
                    other,
                    out _,
                    out _,
                    out var distance,
                    xfA,
                    xfB,
                    fixtureA,
                    fixtureB))
            {
                inRange = false;
            }
            // Overlap, early out and no raycast.
            else if (overlapCheck && distance.Equals(0f))
            {
                return true;
            }
            // Entity can bypass range checks.
            else if (!ShouldCheckAccess(origin))
            {
                return true;
            }
            // Out of range so don't raycast.
            else if (distance > range)
            {
                inRange = false;
            }
            else
            {
                // We'll still do the raycast from the centres but we'll bump the range as we know they're in range.
                originPos = _transform.GetMapCoordinates(origin, xform: origin.Comp);
                range = (originPos.Position - targetPos.Position).Length();
            }
        }
        // No fixtures, e.g. wallmounts.
        else
        {
            originPos = _transform.GetMapCoordinates(origin, origin);
            var otherParent = (other.Comp ?? Transform(other)).ParentUid;
            targetRot = otherParent.IsValid() ? Transform(otherParent).LocalRotation + otherAngle : otherAngle;
        }

        // Do a raycast to check if relevant
        if (inRange)
        {
            var rayPredicate = GetPredicate(originPos, other, targetPos, targetRot, collisionMask, combinedPredicate);
            inRange = InRangeUnobstructed(originPos, targetPos, range, collisionMask, rayPredicate, ShouldCheckAccess(origin));
        }

        if (!inRange && popup && _gameTiming.IsFirstTimePredicted)
        {
            var message = Loc.GetString("interaction-system-user-interaction-cannot-reach");
            _popupSystem.PopupClient(message, origin, origin);
        }

        return inRange;
    }

    public bool InRangeUnobstructed(
            Entity<TransformComponent?> origin,
            Entity<TransformComponent?> other,
            float range = InteractionRange,
            CollisionGroup collisionMask = InRangeUnobstructedMask,
            Ignored? predicate = null,
            bool popup = false,
            bool overlapCheck = true)
    {
        if (!Resolve(other, ref other.Comp))
            return false;

        var ev = new InRangeOverrideEvent(origin, other);
        RaiseLocalEvent(origin, ref ev);

        if (ev.Handled)
        {
            return ev.InRange;
        }

        return InRangeUnobstructed(origin,
            other,
            other.Comp.Coordinates,
            other.Comp.LocalRotation,
            range,
            collisionMask,
            predicate,
            popup,
            overlapCheck);
    }
}
