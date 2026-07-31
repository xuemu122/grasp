'use strict';

function squaredDistance(a, b) {
    return (Number(a.x) - Number(b.x)) ** 2 + (Number(a.y) - Number(b.y)) ** 2;
}

function orderItemsByNearestPickup(items, { markers, startPoint, placePoint, markerNameForItem }) {
    if (!Array.isArray(items) || items.length < 2 || !startPoint || !placePoint) return items;

    const pending = items.map((item, index) => {
        const markerName = markerNameForItem(item);
        return { item, index, markerName, point: markers.get(markerName) || null };
    });
    // Keep the original order when routing data is incomplete. Reordering only
    // part of a paid order could place products before a later config failure.
    if (pending.some((entry) => !entry.point)) return items;

    const ordered = [];
    let origin = startPoint;
    while (pending.length) {
        pending.sort((a, b) => {
            const distanceDelta = squaredDistance(origin, a.point) - squaredDistance(origin, b.point);
            return distanceDelta || a.index - b.index;
        });
        ordered.push(pending.shift());
        // Every run is one complete bottle cycle: shelf -> place. Therefore
        // each subsequent pickup starts from the configured placement marker.
        origin = placePoint;
    }
    return ordered.map((entry) => entry.item);
}

module.exports = { orderItemsByNearestPickup };
