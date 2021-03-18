//
// Created by bjgilhul on 19/06/19.
// https://github.com/idlebear/VisibilityPolygon
//

#include "VisibilityPolygon.h"

#include "include/decomp/source/decomp/convex_decomposition.hpp"
#include "include/decomp/source/decomp/triangulation.hpp"
#include "include/polyDecomp/include/polyDecomp.h"

#define DEBUG 1

#ifdef DEBUG
#include <iostream>

// Format total borrowed from:
// https://stackoverflow.com/questions/35811230/c-variable-amount-of-arguments-with-formatting-for-stdcout
template <typename T>
void disp(std::ostream& out, T arg) {
    out << arg;
}

// recursively displays every arg
template <typename T, typename ... U>
void disp(std::ostream& out, T arg, U ... args) {
    disp(out, arg);
    disp(out, args...);
}

/* fatal displays its args to std::cout, preceded with "FATAL " and followed
 * by a newline.
 * It then does some cleanup and exits
 */
template<typename ... T>
void print_string(T ... args) {
    disp(std::cout, args...);
    std::cout << std::endl;
}

#define  DEBUG_PRINT(...)

#else
#define  DEBUG_PRINT(...)
#endif



#include <memory>

namespace Visibility {

    void
        quickhullRecursive(Polygon& hull, const vector<Point>& pts, const Point& A, const Point& B) {
        if (pts.empty()) {
            return;
        }

        // 4: Find the furthest point from the line and add it to our list
        Segment s(A, B);

        double maxDist = bg::distance(pts[0], s);
        int maxI = 0;
        for (int i = 1; i < pts.size(); i++) {
            auto d = bg::distance(pts[i], s);
            if (d > maxDist) {
                maxDist = d;
                maxI = i;
            }
        }

        // the correct insertion point into the polygon is after point A
        for (auto it = bg::exterior_ring(hull).begin(); it != bg::exterior_ring(hull).end(); it++) {
            if (*it == A) {
                bg::exterior_ring(hull).insert(it + 1, pts[maxI]);
                break;
            }
        }

        // 5: make a triangle out of our points and ignore any contained within

        // 5b: find the nearest point on the segment to define a perpendicular bisector for splitting the
        //     remaining points
        auto nearest = nearestPoint(s, pts[maxI]);
        Segment sp(nearest, pts[maxI]);

        vector<Point> right;
        vector<Point> left;
        for (int i = 0; i < pts.size(); i++) {
            if (i == maxI) {
                continue;
            }
            //            if( !bg::within( pts[i], tri ) ) {
            if (side({ A, pts[maxI] }, pts[i]) > 0 ||
                side({ pts[maxI], B }, pts[i]) > 0) {
                if (side(sp, pts[i]) > 0) {
                    right.emplace_back(pts[i]);
                }
                else {
                    left.emplace_back(pts[i]);
                }
            }
        }

        // 5: Make a recursive call to find the next points
        quickhullRecursive(hull, right, A, pts[maxI]);
        quickhullRecursive(hull, left, pts[maxI], B);
    }

    // Ref: https://en.wikipedia.org/wiki/Quickhull
    Polygon
        quickhull(const vector<Point>& pts) {
        if (pts.empty()) {
            return {};
        }
        else if (pts.size() == 1) {
            return { { pts[0] } };
        }
        else if (pts.size() == 2) {
            return { { pts[0], pts[1], pts[0] } };
        }

        // 1: Find the points with min and max x as they must be on the hull
        double maxX = pts[0].x();
        int maxIndex = 0;
        double minX = pts[0].x();
        int minIndex = 0;

        for (int i = 1; i < pts.size(); i++) {
            auto x = pts[i].x();
            if (x < minX) {
                minX = x;
                minIndex = i;
            }
            if (x > maxX) {
                maxX = x;
                maxIndex = i;
            }
        }
        // If they're in a column, odd results can happen
        if (maxIndex == minIndex) {
            double maxY = pts[0].y();
            maxIndex = 0;
            double minY = pts[0].y();
            minIndex = 0;
            for (int i = 1; i < pts.size(); i++) {
                auto y = pts[i].y();
                if (y < minY) {
                    minY = y;
                    minIndex = i;
                }
                if (y > maxY) {
                    maxY = y;
                    maxIndex = i;
                }
            }
        }

        Polygon hull{ {pts[minIndex], pts[maxIndex], pts[minIndex]} };

        // 2: use the line to divide the points
        Segment s(pts[minIndex], pts[maxIndex]);

        vector<Point> left;
        vector<Point> right;
        for (int i = 0; i < pts.size(); i++) {
            if (i == minIndex || i == maxIndex) {
                continue;
            }
            auto dir = side(s, pts[i]);
            // check which side of the line for each point -- arbitrarily selecting +ve is right
            // if the point is on the line (==0), then ignore it, it's not on the hull...
            if (dir > 0) {
                right.emplace_back(pts[i]);
            }
            else if (dir < 0) {
                left.emplace_back(pts[i]);
            }
        }

        // 3: Recursive call to further process the remaining points divide and conquer style...
        quickhullRecursive(hull, right, s.first, s.second);
        quickhullRecursive(hull, left, s.second, s.first);

        // If all went well, the hull should now be fully derived
        return hull;
    }



    vector<Segment>
        convertToSegments(const Polygon& poly) {
        vector<Segment> segments;
        vector<Point> points = convertToExteriorPoints(poly);

        auto n = points.size();
        // Note: polygons are closed with the same point at the start and end of the ring
        for (int j = 0; j < n - 1; ++j) {
            segments.emplace_back(Segment(points[j], points[j + 1]));
        }
        return segments;
    }

    vector<Point>
        convertToExteriorPoints(const Polygon& poly) {
        vector<Point> res;

        for (auto const& point : bg::exterior_ring(poly)) {
            res.emplace_back(point);
        }
        return res;
    }

    vector<Point>
        convertToPoints(const Segment& segment) {
        vector<Point> res;
        res.emplace_back(segment.first);
        res.emplace_back(segment.second);
        return res;
    }

    vector<Segment>
        convertToSegments(const PolyLine& line) {
        vector<Segment> segments;

        int n = line.size();
        for (int i = 0; i < n - 1; i++) {
            segments.emplace_back(Segment(line[i], line[i + 1]));
        }
        return segments;
    }


    MultiPolygon
        expand(const PolyLine& line, double distance, int pointsPerCircle) {
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
        boost::geometry::strategy::buffer::join_round join_strategy(pointsPerCircle);
        boost::geometry::strategy::buffer::end_flat end_strategy; // (pointsPerCircle);
        boost::geometry::strategy::buffer::point_circle point_strategy(pointsPerCircle);
        boost::geometry::strategy::buffer::side_straight side_strategy;

        // Create the buffer of a linestring
        MultiPolygon result;
        boost::geometry::buffer(line, result,
            distance_strategy, side_strategy,
            join_strategy, end_strategy, point_strategy);
        return result;
    }



    MultiPolygon
        expand(const Polygon& poly, double distance, int pointsPerCircle) {
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
        boost::geometry::strategy::buffer::join_round join_strategy(pointsPerCircle);
        boost::geometry::strategy::buffer::end_round end_strategy(pointsPerCircle);
        boost::geometry::strategy::buffer::point_circle point_strategy(pointsPerCircle);
        boost::geometry::strategy::buffer::side_straight side_strategy;

        // Create the buffer of a linestring
        MultiPolygon result;
        boost::geometry::buffer(poly, result,
            distance_strategy, side_strategy,
            join_strategy, end_strategy, point_strategy);
        return result;
    }



    Polygon
        compute(const Point& position, const vector<Segment>& segments) {
        vector<Segment> bounded;
        auto minX = position.x();
        auto minY = position.y();
        auto maxX = position.x();
        auto maxY = position.y();
        for (auto const& segment : segments) {

            auto pts = convertToPoints(segment);
            for (auto const& pt : pts) {
                minX = min(minX, pt.x());
                minY = min(minY, pt.y());
                maxX = max(maxX, pt.x());
                maxY = max(maxY, pt.y());
            }
            bounded.emplace_back(segment);
        }

        --minX;
        --minY;
        ++maxX;
        ++maxY;

        bounded.emplace_back(Segment(Point(minX, minY), Point(maxX, minY)));
        bounded.emplace_back(Segment(Point(maxX, minY), Point(maxX, maxY)));
        bounded.emplace_back(Segment(Point(maxX, maxY), Point(minX, maxY)));
        bounded.emplace_back(Segment(Point(minX, maxY), Point(minX, minY)));

        Polygon polygon;

        auto sorted = sortPoints(position, bounded);

        // TODO: At some point this should be converted to use the STL heap (?)
        vector<int> heap;
        auto vertex = Point(position.x() + 1, position.y());
        vector<int> map(bounded.size(), -1);

        int c = 0;
        for (auto const& segment : bounded) {
            auto a1 = angle(segment.first, position);
            auto a2 = angle(segment.second, position);
            bool active = false;
            if (a1 > -M_PI && a1 <= 0 && a2 <= M_PI && a2 >= 0 && a2 - a1 > M_PI) {
                active = true;
            }
            if (a2 > -M_PI && a2 <= 0 && a1 <= M_PI && a1 >= 0 && a1 - a2 > M_PI) {
                active = true;
            }
            if (active) {
                insert(c, heap, position, bounded, vertex, map);
            }
            c++;
        }

        for (int i = 0; i < sorted.size();) {
            bool extend = false;
            bool shorten = false;
            auto orig = i;
            vertex = sorted[i].j ? bounded[sorted[i].i].second : bounded[sorted[i].i].first;
            int old_segment = heap[0];
            do {
                if (map[sorted[i].i] != -1) {
                    if (sorted[i].i == old_segment) {
                        extend = true;
                        vertex = sorted[i].j ? bounded[sorted[i].i].second : bounded[sorted[i].i].first;
                    }
                    remove(map[sorted[i].i], heap, position, bounded, vertex, map);
                }
                else {
                    insert(sorted[i].i, heap, position, bounded, vertex, map);
                    if (heap[0] != old_segment) {
                        shorten = true;
                    }
                }
                ++i;
                if (i == sorted.size()) {
                    break;
                }
            } while (sorted[i].a < sorted[orig].a + VISIBILITY_POLYGON_EPSILON);

            if (extend) {
                bg::append(polygon, vertex);
                Point cur;
                if (intersectLines(bounded[heap[0]], Segment(position, vertex), cur)) {
                    if (cur != vertex) {
                        bg::append(polygon, cur);
                    }
                }
            }
            else if (shorten) {
                Point pt;
                if (intersectLines(bounded[old_segment], Segment(position, vertex), pt)) {
                    bg::append(polygon, pt);
                }
                if (intersectLines(bounded[heap[0]], Segment(position, vertex), pt)) {
                    bg::append(polygon, pt);
                }
            }
        }
        bg::correct(polygon);
        return polygon;
    }

    Polygon
        computeViewport(const Point& position, const vector <Segment>& segments, const Point& viewportMinCorner,
            const Point& viewportMaxCorner) {
        vector <Segment> brokenSegments;
        Point viewport[4] = { Point(viewportMinCorner.x(), viewportMinCorner.y()),
                           Point(viewportMaxCorner.x(), viewportMinCorner.y()),
                           Point(viewportMaxCorner.x(), viewportMaxCorner.y()),
                           Point(viewportMinCorner.x(), viewportMaxCorner.y()) };

        for (auto const& segment : segments) {
            if (segment.first.x() < viewportMinCorner.x() && segment.second.x() < viewportMinCorner.x()) { continue; }
            if (segment.first.y() < viewportMinCorner.y() && segment.second.y() < viewportMinCorner.y()) { continue; }
            if (segment.first.x() > viewportMaxCorner.x() && segment.second.x() > viewportMaxCorner.x()) { continue; }
            if (segment.first.y() > viewportMaxCorner.y() && segment.second.y() > viewportMaxCorner.y()) { continue; }

            vector<Point> intersections;
            for (int j = 0; j < 4; ++j) {
                int k = (j + 1) % 4;
                Point pt;
                if (intersectSegments(segment, Segment(viewport[j], viewport[k]), pt)) {
                    if (pt != segment.first && pt != segment.second) {
                        intersections.push_back(pt);
                    }
                }
            }

            Point start(segment.first);
            while (!intersections.empty()) {
                int endIndex = 0;
                double endDis = bg::distance(start, intersections[0]);
                for (int j = 1; j < intersections.size(); ++j) {
                    double dis = bg::distance(start, intersections[j]);
                    if (dis < endDis) {
                        endDis = dis;
                        endIndex = j;
                    }
                }
                brokenSegments.emplace_back(Segment(start, intersections[endIndex]));
                start = intersections[endIndex];
                intersections.erase(intersections.begin() + endIndex);
            }
            brokenSegments.emplace_back(Segment(start, segment.second));
        }

        vector<Segment> viewportSegments;
        for (auto const& segment : brokenSegments) {
            if (inViewport(segment.first, viewportMinCorner, viewportMaxCorner) &&
                inViewport(segment.second, viewportMinCorner, viewportMaxCorner)) {
                viewportSegments.push_back(segment);
            }
        }
        auto eps = VISIBILITY_POLYGON_EPSILON * 10.0;
        viewportSegments.emplace_back(Segment(Point(viewportMinCorner.x() - eps, viewportMinCorner.y() - eps),
            Point(viewportMaxCorner.x() + eps, viewportMinCorner.y() - eps)));
        viewportSegments.emplace_back(Segment(Point(viewportMaxCorner.x() + eps, viewportMinCorner.y() - eps),
            Point(viewportMaxCorner.x() + eps, viewportMaxCorner.y() + eps)));
        viewportSegments.emplace_back(Segment(Point(viewportMaxCorner.x() + eps, viewportMaxCorner.y() + eps),
            Point(viewportMinCorner.x() - eps, viewportMaxCorner.y() + eps)));
        viewportSegments.emplace_back(Segment(Point(viewportMinCorner.x() - eps, viewportMaxCorner.y() + eps),
            Point(viewportMinCorner.x() - eps, viewportMinCorner.y() - eps)));
        return compute(position, viewportSegments);
    }

    vector<Segment> breakIntersections(const vector<Segment>& segments) {
        vector<Segment> output;
        auto n = segments.size();
        for (int i = 0; i < n; ++i) {
            vector<Point> intersections;
            for (int j = 0; j < n; ++j) {
                if (i == j) {
                    continue;
                }
                Point pt;
                if (intersectSegments(segments[i], segments[j], pt)) {
                    if (pt != segments[i].first && pt != segments[i].second) {
                        intersections.emplace_back(pt);
                    }
                }
            }
            Point start = segments[i].first;
            while (!intersections.empty()) {
                int endIndex = 0;
                double endDis = bg::distance(start, intersections[0]);
                for (int j = 1; j < intersections.size(); ++j) {
                    double dis = bg::distance(start, intersections[j]);
                    if (dis < endDis) {
                        endDis = dis;
                        endIndex = j;
                    }
                }
                output.emplace_back(Segment(start, intersections[endIndex]));
                start = intersections[endIndex];
                intersections.erase(intersections.begin() + endIndex);
            }
            output.emplace_back(Segment(start, segments[i].second));
        }
        return output;
    };

    void remove(int index, vector<int>& heap, const Point& position, const vector<Segment>& segments,
        const Point& destination, vector<int>& map) {
        map[heap[index]] = -1;
        if (index == heap.size() - 1) {
            heap.pop_back();
            return;
        }
        heap[index] = heap.back();
        heap.pop_back();
        map[heap[index]] = index;
        int cur = index;
        int parent = heapParent(cur);
        if (cur != 0 && lessThan(heap[cur], heap[parent], position, segments, destination)) {
            while (cur > 0) {
                parent = heapParent(cur);
                if (!lessThan(heap[cur], heap[parent], position, segments, destination)) {
                    break;
                }
                map[heap[parent]] = cur;
                map[heap[cur]] = parent;
                int temp = heap[cur];
                heap[cur] = heap[parent];
                heap[parent] = temp;
                cur = parent;
            }
        }
        else {
            while (true) {
                int left = heapChild(cur);
                int right = left + 1;
                if (left < heap.size() && lessThan(heap[left], heap[cur], position, segments, destination) &&
                    (right == heap.size() || lessThan(heap[left], heap[right], position, segments, destination))) {
                    map[heap[left]] = cur;
                    map[heap[cur]] = left;
                    int temp = heap[left];
                    heap[left] = heap[cur];
                    heap[cur] = temp;
                    cur = left;
                }
                else if (right < heap.size() && lessThan(heap[right], heap[cur], position, segments, destination)) {
                    map[heap[right]] = cur;
                    map[heap[cur]] = right;
                    int temp = heap[right];
                    heap[right] = heap[cur];
                    heap[cur] = temp;
                    cur = right;
                }
                else {
                    break;
                }
            }
        }
    }

    void
        insert(int index, vector<int>& heap, const Point& position, const vector<Segment>& segments,
            const Point& destination, vector<int>& map) {
        Point intersection;
        if (intersectLines(segments[index], Segment(position, destination), intersection)) {
            int cur = heap.size();
            heap.push_back(index);
            map[index] = cur;
            while (cur > 0) {
                int parent = heapParent(cur);
                if (!lessThan(heap[cur], heap[parent], position, segments, destination)) {
                    break;
                }
                map[heap[parent]] = cur;
                map[heap[cur]] = parent;
                int temp = heap[cur];
                heap[cur] = heap[parent];
                heap[parent] = temp;
                cur = parent;
            }
        }
    }

    bool lessThan(int i1, int i2, const Point& position, const vector<Segment>& segments, const Point& destination) {

        Segment s1 = segments[i1];
        Segment s2 = segments[i2];

        Segment destSegment = Segment(position, destination);
        Point inter1;
        if (!intersectLines(s1, destSegment, inter1)) {
            return false;
        }
        Point inter2;
        if (!intersectLines(s2, destSegment, inter2)) {
            return true;
        }
        if (inter1 != inter2) {
            return bg::distance(inter1, position) < bg::distance(inter2, position);
        }
        Point end1 = s1.first;
        if (inter1 == s1.first) {
            end1 = s1.second;
        }
        Point end2 = s2.first;
        if (inter2 == s2.first) {
            end2 = s2.second;
        }
        double a1 = angle2(end1, inter1, position);
        double a2 = angle2(end2, inter2, position);
        if (a1 < M_PI) {
            if (a2 > M_PI) {
                return true;
            }
            return a2 < a1;
        }
        return a1 < a2;
    }


    vector<PointIndex> sortPoints(const Point& position, const vector<Segment>& segments) {
        vector<PointIndex> points;
        points.reserve(segments.size() * 2);

        for (int i = 0; i < segments.size(); ++i) {
            auto pts = convertToPoints(segments[i]);
            for (int j = 0; j < 2; ++j) {
                double a = angle(pts[j], position);
                points.emplace_back(PointIndex{ i, j, a });
            }
        }
        sort(points.begin(), points.end(), [](const PointIndex& a, const PointIndex& b) {
            return a.a < b.a;
            });
        return points;
    };

    int
        findIndex(const vector<decomp::Point>& pts, const decomp::Point& targetPt) {
        int i = 0;
        for (auto const& pt : pts) {
            if (pt == targetPt) {
                return i;
            }
            i++;
        }
        return -1;
    }

    /*
    vector<Polygon>
        alternateDecompose(const Polygon& polygon) {

        Polygon_Decomposition::Polygon target;

        // Add all the points from the exterior ring to point list
        for (auto it = bg::exterior_ring(polygon).rbegin(); it != bg::exterior_ring(polygon).rend() - 1; it++) {
            Polygon_Decomposition::Point pt((*it).x(), (*it).y());

            target.emplace_back(pt);
        }

        if (target.size() < 3) {
            return { polygon };
        }

        vector<Polygon_Decomposition::Polygon> decomposition;
        Polygon_Decomposition::decomposePoly(target, decomposition);

        vector<Polygon> result;
        // convert the resulting index lists back into polygons...
        for (auto const& polyList : decomposition) {
            if (polyList.begin() != polyList.end()) {
                Polygon convexPoly;
                for (auto it = polyList.begin(); it != polyList.end(); it++) {
                    auto p = *it;
                    addPoint(convexPoly, { p.x, p.y });
                }
                // close the poly
                auto p = *(polyList.begin());
                addPoint(convexPoly, { p.x, p.y });
                result.emplace_back(convexPoly);
            }
        }

        return result;
    }
    */

    /*
    vector<Polygon>
        decompose(const Polygon& polygon) {
        vector<decomp::Point> dPoints;
        decomp::IndexList exteriorRing;
        vector<decomp::IndexList> holes;
        vector<Polygon> result;

        if (!bg::is_simple(polygon)) {
            return result;
        }

        // Add all the points from the exterior ring to point list
        for (auto it = bg::exterior_ring(polygon).rbegin(); it != bg::exterior_ring(polygon).rend() - 1; it++) {
            decomp::Point pt((*it).x(), (*it).y());
            auto index = findIndex(dPoints, pt);
            if (index == -1) {
                dPoints.emplace_back(pt);
                index = dPoints.size() - 1;
            }
            exteriorRing.emplace_back(index);
        }

        // Need to have at least 3 exterior points...
        if (dPoints.size() < 3) {
            return { polygon };
        }

        // Now all the interior rings
        for (auto const& inner : bg::interior_rings(polygon)) {
            decomp::IndexList hole;
            for (auto it = inner.rbegin(); it != inner.rend() - 1; it++) {
                decomp::Point pt((*it).x(), (*it).y());
                auto index = findIndex(dPoints, pt);
                if (index == -1) {
                    dPoints.emplace_back(pt);
                    index = dPoints.size() - 1;
                }
                hole.emplace_back(index);
            }
            holes.emplace_back(hole);
        }


        auto decomposition = decomp::decompose(dPoints, exteriorRing, holes);

        // convert the resulting index lists back into polygons...
        for (auto const& polyList : decomposition) {
            if (polyList.begin() != polyList.end()) {
                Polygon convexPoly;
                for (auto it = polyList.begin(); it != polyList.end(); it++) {
                    auto i = *it;
                    addPoint(convexPoly, { dPoints[i][0], dPoints[i][1] });
                }
                // close the poly
                auto i = *(polyList.begin());
                addPoint(convexPoly, { dPoints[i][0], dPoints[i][1] });
                result.emplace_back(convexPoly);
            }
        }

        return result;
    }
    */

    //
    // Based on the algorithm published by Pirzadeh's 1999 thesis:
    // @phdthesis{pirzadeh1999computational,
    //   title={Computational geometry with the rotating calipers},
    //   author={Pirzadeh, Hormoz},
    //   year={1999},
    //   school={McGill University}
    // }
    vector<pair<int, int>>
        findAntipodals(const Polygon& poly) {
        auto const& points = bg::exterior_ring(poly);
        vector<pair<int, int>> antipods;

        auto n = points.size() - 1;  // skip the extra (closing) pt
        if (n < 3) {
            // Need to have at least three points to have anything useful
            return {};
        }

        // Find the initial pair -- basically move around the polygon until the area of
        // the triangle stops growing
        auto pStart = 0;
        auto p = n - 1;
        auto q = (p + 1) % n;

        while (epsilonGreaterThan(area(points[p], points[(p + 1) % n], points[(q + 1) % n]),
            area(points[p], points[(p + 1) % n], points[q]))) {
            q = (q + 1) % n;
        }

        auto qStart = q;
        auto pEnd = n - 1;
        while (q != pStart) {
            p = (p + 1) % n;
            antipods.emplace_back(p, q);
            while (epsilonGreaterThan(area(points[p], points[(p + 1) % n], points[(q + 1) % n]),
                area(points[p], points[(p + 1) % n], points[q]))) {
                q = (q + 1) % n;
                if ( /*p == qStart || */ q == pStart) {
                    // all done here
                    return antipods;
                }
                antipods.emplace_back(p, q);
            }
            if (epsilonEqual(area(points[p], points[(p + 1) % n], points[(q + 1) % n]),
                area(points[p], points[(p + 1) % n], points[q]))) {
                if (p != qStart && q != pStart) {
                    antipods.emplace_back(p, (q + 1) % n);
                }
                else {
                    // antipods.emplace_back( (p+1)%n, q );
                }
            }
        }

        return antipods;
    }

    vector<tuple<int, int, int, double>>
        findHeights(const Polygon& poly) {
        auto const& points = bg::exterior_ring(poly);
        vector<tuple<int, int, int, double >> heights;

        auto n = points.size() - 1;  // skip the extra (closing) pt
        if (n < 3) {
            // Need to have at least three points to have anything useful
            return {};
        }

        auto p = 0;
        auto q = p + 1;
        while (p < n) {
            auto currentArea = area(points[p], points[(p + 1) % n], points[q]);
            while (epsilonGreaterThan(area(points[p], points[(p + 1) % n], points[(q + 1) % n]), currentArea)) {
                q = (q + 1) % n;
                currentArea = area(points[p], points[(p + 1) % n], points[q]);
            }
            auto h = currentArea * 2.0 / bg::distance(points[p], points[p + 1]);
            if (h > 0) {
                auto t = make_tuple(p, p + 1, q, h);
                heights.emplace_back(t);
            }
            p++;
        }
        return heights;
    }



}