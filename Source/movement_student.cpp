/* Copyright Steve Rabin, 2012.
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */

#include <Stdafx.h>
#include <queue>

typedef struct node
{
public:
    int row, col;
    int gx, fx;
    int hx;
    std::pair<int, int> parent;
}Node;

struct my_comp
{
    bool operator()(Node a, Node b)
    {
        if (a.fx > b.fx)
            return true;
        else if (a.fx == b.fx)
        {
            if (a.gx < b.gx)
                return true;
            else
                return false;
        }
        return false;
    }
};

enum HEURISTIC
{
    EUCLIDEAN = 0,
    OTILE = 1,
    CHEVYSHEV = 2,
    MANHATTAN = 3
};

int Movement::GetHeuristic(int _startRow, int _startCol, int _targetRow, int _targetCol, enum HEURISTIC _h, float _weight)
{
    switch (_h)
    {
    case HEURISTIC::EUCLIDEAN:
        return int((sqrt(pow(_targetRow - _startRow, 2) + pow(_targetCol - _startCol, 2)) * 10 * _weight));
    case HEURISTIC::OTILE:
        return max(abs(_targetRow - _startRow) * 10 * _weight, abs(_targetCol - _startCol) * 10 * _weight) + (sqrt(2) - 1) * 10 * min(abs(_targetRow - _startRow) * 10 * _weight, abs(_targetCol - _startCol) * 10 * _weight);
    case HEURISTIC::CHEVYSHEV:
        return max(abs(_targetRow - _startRow) * 10 * _weight, abs(_targetCol - _startCol) * 10 * _weight);
    case HEURISTIC::MANHATTAN:
        return (abs(_targetRow - _startRow) + abs(_targetCol - _startCol)) * 10 * _weight;
    }
}

void DrawPath(const std::vector<Node>& _path, DebugDrawingColor _eColor)
{
    for (int i = 0; i < _path.size() - 1; i++)
    {
        g_terrain.SetColor(_path[i].row, _path[i].col, _eColor);
    }
}

void Movement::DrawParentPathAndMove(const std::vector<std::pair<int, int>>& _path)
{
    for (int i = _path.size() - 1; i >= 0; i--)
    {
        D3DXVECTOR3 spot = g_terrain.GetCoordinates(_path[i].first, _path[i].second);
        m_waypointList.push_back(spot);
        g_terrain.SetColor(_path[i].first, _path[i].second, DEBUG_COLOR_YELLOW);
    }
}

//ºê·¹Á¨Çð ¾Ë°í¸®Áò
bool GoStraight(int startRow, int startCol, int endRow, int endCol)
{
    int delta_col = abs(endCol - startCol);
    int delta_row = abs(endRow - startRow);
    int col = (startCol < endCol) ? 1 : -1;
    int row = (startRow < endRow) ? 1 : -1;
    int err = delta_col - delta_row;

    int maxSteps = max(delta_col, delta_row);


    for (int i = 0; i <= maxSteps; i++)
    {

        if (g_terrain.IsWall(startRow, startCol))
            return false;


        if (startRow == endRow && startCol == endCol)
            break;


        int e2 = err * 2;

        if (e2 > -delta_row)
        {
            err -= delta_row;
            startCol += col;
        }
        if (e2 < delta_col)
        {
            err += delta_col;
            startRow += row;
        }
    }
    return true;
}

void erase_element(std::vector<std::pair<int, int>>* _vec, int _idx)
{
    for (int i = 0; i < _vec->size();)
    {
        if (i == _idx)
        {
            _vec->erase(_vec->begin() + i);
            break;
        }
        else
        {
            i++;
        }
    }
}

void SmoothPath(std::vector<std::pair<int, int>>& _path)
{
    if (_path.size() < 3)
        return;

    std::vector<std::pair<int, int>> smoothPath;
    smoothPath.push_back(_path[0]);

    for (size_t i = 1; i < _path.size() - 1; ++i)
    {
        auto previous_point = _path[i - 1];
        auto current_point = _path[i];
        auto next_point = _path[i + 1];

        float newX = (previous_point.first + current_point.first + next_point.first) / 3.0f;
        float newY = (previous_point.second + current_point.second + next_point.second) / 3.0f;

        smoothPath.push_back({ (int)newX,(int)newY });
    }

    smoothPath.push_back(_path[_path.size() - 1]);
    _path = smoothPath;
}

std::vector<D3DXVECTOR3> CatmullRom(const std::vector<D3DXVECTOR3>& _points, int _segment)
{
    std::vector<D3DXVECTOR3> smoothPth;

    if (_points.size() < 4)
    {

        return smoothPth;
    }

    for (size_t i = 1; i < _points.size() - 2; i++)
    {
        D3DXVECTOR3 p0 = _points[i - 1];
        D3DXVECTOR3 p1 = _points[i];
        D3DXVECTOR3 p2 = _points[i + 1];
        D3DXVECTOR3 p3 = _points[i + 2];

        for (int j = 0; j <= _segment; j++)
        {
            float t = float(j) / _segment;
            D3DXVECTOR3 point;
            D3DXVec3CatmullRom(&point, &p0, &p1, &p2, &p3, t);
            smoothPth.push_back(point);
        }
    }

    return smoothPth;
}

std::vector<D3DXVECTOR3> SmoothPathWithCatmullRom(const std::vector<std::pair<int, int>>& _path, int _segment)
{
    std::vector<D3DXVECTOR3> controlPoints;
    std::vector<D3DXVECTOR3> smoothedPath;

    for (int i = 0; i < _path.size(); i++)
    {
        controlPoints.push_back(g_terrain.GetCoordinates(_path[i].first, _path[i].second));
    }
    smoothedPath = CatmullRom(controlPoints, _segment);
    return smoothedPath;
}



bool Movement::ComputePath(int r, int c, bool newRequest)
{
    static std::priority_queue<Node, std::vector<Node>, my_comp> open_list;
    static std::vector<Node> close_list;
    static std::vector<Node> blue_list;

    static bool close_grid[1000][1000] = { false, };

    m_goal = g_terrain.GetCoordinates(r, c);

    int max_RowCol = g_terrain.GetWidth();
    int target_Row, target_Col;

    D3DXVECTOR3 target = m_goal;
    g_terrain.GetRowColumn(&target, &target_Row, &target_Col);

    if (newRequest)
    {

        m_movementMode = MOVEMENT_WAYPOINT_LIST;

        int curR, curC;
        D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
        g_terrain.GetRowColumn(&cur, &curR, &curC);

        m_waypointList.clear();
        m_waypointList.push_back(cur);

        int start_Row, start_Col;

        g_terrain.GetRowColumn(&cur, &start_Row, &start_Col);

        Node start_Node;
        start_Node.row = start_Row;
        start_Node.col = start_Col;
        start_Node.gx = 0;
        start_Node.parent = { start_Node.row, start_Node.col };
        start_Node.hx = GetHeuristic(start_Row, start_Col, target_Row, target_Col, HEURISTIC(GetHeuristicCalc()), GetHeuristicWeight());
        start_Node.fx = start_Node.gx + start_Node.hx;

        close_grid[start_Row][start_Col] = true;
        open_list.push(start_Node);
        blue_list.push_back(start_Node);
        if (m_straightline)
        {
            if (GoStraight(curR, curC, target_Row, target_Col))
            {
                D3DXVECTOR3 spot = { target.x, target.y, target.z };
                m_waypointList.push_back(spot);
                return true;
            }
        }
    }


    bool useAStar = true;


    if (useAStar)
    {


        while (!open_list.empty())
        {
            Node add_node;
            //                   ¡è(0)   ¡æ(1)   ¡é(2)   ¡ç(3)  ¢Ö(4)   ¢Ù(5)  ¢×(6)  ¢Ø(7)
            int direct_Row[8] = { 1,     0,    -1,     0,    1,     -1,    -1,     1 };
            int direct_Col[8] = { 0,     1,     0,    -1,    1,      1,    -1,    -1 };

            int cur_Row = open_list.top().row;
            int cur_Col = open_list.top().col;
            int cur_gx = open_list.top().gx;
            close_list.push_back(open_list.top());
            open_list.pop();

            if (cur_Row == r && cur_Col == c)
            {
                std::vector<std::pair<int, int>> result_path;
                int parent_row = close_list[close_list.size() - 1].row;
                int parent_col = close_list[close_list.size() - 1].col;
                result_path.push_back({ close_list[close_list.size() - 1].row, close_list[close_list.size() - 1].col });
                for (int i = close_list.size() - 1; i >= 0; i--)
                {
                    if (parent_row == close_list[i].row && parent_col == close_list[i].col)
                    {
                        parent_row = close_list[i].parent.first;
                        parent_col = close_list[i].parent.second;
                        result_path.push_back({ parent_row, parent_col });
                    }
                }

                std::vector<std::pair<int, int>> rubberband_path;
                if (m_rubberband)
                {
                    for (int i = 0; i < result_path.size(); i++)
                        rubberband_path.push_back({ result_path[i].first,result_path[i].second });

                    for (int i = rubberband_path.size() - 1; i >= 0; i--)
                    {
                        if (i <= 2)
                        {
                            break;
                        }
                        int n1_row = rubberband_path[i].first;
                        int n1_col = rubberband_path[i].second;
                        int n2_row = rubberband_path[i - 1].first;
                        int n2_col = rubberband_path[i - 1].second;
                        int n3_row = rubberband_path[i - 2].first;
                        int n3_col = rubberband_path[i - 2].second;
                        if (GoStraight(n1_row, n1_col, n3_row, n3_col))
                        {
                            erase_element(&rubberband_path, i - 1);
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                std::vector<D3DXVECTOR3> smooth_path;

                DrawPath(blue_list, DEBUG_COLOR_BLUE);
                DrawPath(close_list, DEBUG_COLOR_YELLOW);

                if (m_smooth)
                {
                    smooth_path = SmoothPathWithCatmullRom(result_path, 10);
                    for (int i = smooth_path.size() - 1; i >= 0; i--)
                    {
                        D3DXVECTOR3 spot = { smooth_path[i].x, smooth_path[i].y,smooth_path[i].z };
                        m_waypointList.push_back(spot);
                    }
                }
                else if (m_rubberband)
                {
                    DrawParentPathAndMove(rubberband_path);
                    rubberband_path.clear();
                }
                else
                {
                    DrawParentPathAndMove(result_path);
                    result_path.clear();
                    close_list.clear();
                    blue_list.clear();
                }


                //µµÂøÇÏ¸é clear
                result_path.clear();
                close_list.clear();
                blue_list.clear();
                for (int i = 0; i < 1000; i++)
                {
                    for (int j = 0; j < 1000; j++)
                    {
                        close_grid[i][j] = false;
                    }
                }
                while (!open_list.empty())
                    open_list.pop();
                return true;
            }

            for (int i = 0; i < 8; i++)
            {
                int next_Row = cur_Row + direct_Row[i];
                int next_Col = cur_Col + direct_Col[i];

                if (next_Row < 0 || next_Col < 0)
                    continue;
                if (next_Row >= max_RowCol || next_Col >= max_RowCol)
                    continue;
                if (g_terrain.IsWall(next_Row, next_Col))
                    continue;
                if (close_grid[next_Row][next_Col])
                    continue;

                if (i >= 4)
                {
                    int adj_Row1 = cur_Row + direct_Row[i];
                    int adj_Col1 = cur_Col;
                    int adj_Row2 = cur_Row;
                    int adj_Col2 = cur_Col + direct_Col[i];
                    if (g_terrain.IsWall(adj_Row1, adj_Col1) || g_terrain.IsWall(adj_Row2, adj_Col2))
                        continue;
                }

                add_node.row = next_Row;
                add_node.col = next_Col;

                if (i < 4)
                    add_node.gx = cur_gx + 10;
                else
                    add_node.gx = cur_gx + 14;

                add_node.hx = GetHeuristic(next_Row, next_Col, target_Row, target_Col, (HEURISTIC)GetHeuristicCalc(), GetHeuristicWeight());
                add_node.fx = add_node.gx + add_node.hx;

                add_node.parent = { cur_Row, cur_Col };
                close_grid[next_Row][next_Col] = true;

                blue_list.push_back(add_node);
                open_list.push(add_node);
            }
            if (GetSingleStep())
            {
                DrawPath(blue_list, DEBUG_COLOR_BLUE);
                DrawPath(close_list, DEBUG_COLOR_YELLOW);
                return false;
            }
        }
        return true;
    }
    else
    {
        int curR, curC;
        D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
        g_terrain.GetRowColumn(&cur, &curR, &curC);

        m_waypointList.clear();
        m_waypointList.push_back(cur);

        int countdown = 100;
        while (curR != r || curC != c)
        {
            if (countdown-- < 0) { break; }

            if (curC == c || (curR != r && rand() % 2 == 0))
            {
                int last = curR;
                if (curR < r) { curR++; }
                else { curR--; }

                if (g_terrain.IsWall(curR, curC))
                {
                    curR = last;
                    continue;
                }
            }
            else
            {
                int last = curC;
                if (curC < c) { curC++; }
                else { curC--; }

                if (g_terrain.IsWall(curR, curC))
                {
                    curC = last;
                    continue;
                }
            }

            D3DXVECTOR3 spot = g_terrain.GetCoordinates(curR, curC);
            m_waypointList.push_back(spot);
            g_terrain.SetColor(curR, curC, DEBUG_COLOR_YELLOW);
        }
        return true;
    }
}