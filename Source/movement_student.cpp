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




bool Movement::ComputePath( int r, int c, bool newRequest )
{
	m_goal = g_terrain.GetCoordinates( r, c );
	m_movementMode = MOVEMENT_WAYPOINT_LIST;

	//Randomly meander toward goal (might get stuck at wall)
	int curR, curC;
	D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
	g_terrain.GetRowColumn(&cur, &curR, &curC);

	m_rubberband = false;

	m_waypointList.clear();
	m_waypointList.push_back(cur);

	// project 2: change this flag to true
	bool useAStar = true;
	int col_cnt = 0;
	/*if (useAStar)
	{
		std::vector<std::pair<int, int>> original_path;
		original_path.push_back(make_pair(0,0));

		Rubberband();
		


		return true;
	}*/

	if( useAStar )
	{		
		std::priority_queue<Node,std::vector<Node>,my_comp> open_list;
		std::vector<Node> close_list;
		std::vector<Node> blue_list;
		std::vector<std::pair<int, int>> result_path;
		std::vector<Node> rubber_path;
		int col_cnt = 0;

		int start_Row, start_Col;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn(&cur, &start_Row, &start_Col);		

		int target_Row, target_Col;
		D3DXVECTOR3 target = m_goal;
		g_terrain.GetRowColumn(&target, &target_Row, &target_Col);

		int max_RowCol = g_terrain.GetWidth();
		bool search[1000][1000] = { false, };

		Node start_Node;
		start_Node.row = start_Row;
		start_Node.col = start_Col;
		start_Node.gx = 0;
		start_Node.parent = { -1,-1 };
		start_Node.hx = (abs(target_Row - start_Row) + abs(target_Col - start_Col)) * 10;
		start_Node.fx = start_Node.gx+start_Node.hx;
		
		search[start_Row][start_Col] = true;
		open_list.push(start_Node);
		blue_list.push_back(start_Node);
		

		while (!open_list.empty())
		{		
			Node add_node;
								// ↑   →    ↓    ←   ↗  ↘   ↙  ↖ 
			int direct_Row[8] = {  1,  0,  -1,   0,  1,  -1,  -1, 1 };
			int direct_Col[8] = {  0,  1,   0,  -1,  1,  1,  -1, -1 };		
		
			int cur_Row = open_list.top().row;
			int cur_Col = open_list.top().col;									
			int cur_Weight = open_list.top().gx;
			close_list.push_back(open_list.top());			


			if (cur_Row == r && cur_Col == c)//찾았당
			{							
				int parent_row = close_list.back().row;
				int parent_col = close_list.back().col;
				while (close_list.size())
				{
					if (parent_row == close_list.back().row && parent_col == close_list.back().col)
					{
						parent_row = close_list.back().parent.first;
						parent_col = close_list.back().parent.second;
						if (parent_row == -1 || parent_col == -1)
							break;
						result_path.push_back({ parent_row, parent_col });												
					}
					close_list.pop_back();										
				}				
				

				if (m_rubberband)
				{
					for (int k = 0; k < rubber_path.size(); k++)
					{
						D3DXVECTOR3 spot = g_terrain.GetCoordinates(rubber_path[k].row, rubber_path[k].col);
						m_waypointList.push_back(spot);
						g_terrain.SetColor(rubber_path[k].row, rubber_path[k].col, DEBUG_COLOR_YELLOW);
					}
					rubber_path.clear();
					return true;
				}
				for (int z = result_path.size(); z--; z > 0)
				{
					D3DXVECTOR3 spot = g_terrain.GetCoordinates(result_path[z].first, result_path[z].second);
					m_waypointList.push_back(spot);
					g_terrain.SetColor(result_path[z].first, result_path[z].second, DEBUG_COLOR_YELLOW);
				}
				result_path.clear();
				rubber_path.clear();
				return true;																				
			}
				
			open_list.pop();

			for (int i = 0; i < 8; i++)
			{				
				int next_Row = cur_Row + direct_Row[i];
				int next_Col = cur_Col + direct_Col[i];
		
				if (next_Row < 0 || next_Col < 0)
					continue;
				if (next_Row >= max_RowCol || next_Col >= max_RowCol)
					continue;
				if(g_terrain.IsWall(next_Row,next_Col))
					continue;
				if (search[next_Row][next_Col])
					continue;								

				//대각선으로 벽이 있을 경우
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
				{
					add_node.gx = cur_Weight + 10;
				}
				else
				{
					add_node.gx = cur_Weight + 14;
				}
				add_node.hx = (abs(target_Row - next_Row) + abs(target_Col - next_Col)) * 10;//맨헤튼				
				add_node.fx = add_node.gx + add_node.hx;
				
				add_node.parent = { cur_Row,cur_Col };
				search[next_Row][next_Col] = true;
				open_list.push(add_node); 				
			}
		}
		return false;
	}
	else
	{	
		//Randomly meander toward goal (might get stuck at wall)
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn( &cur, &curR, &curC );

		m_waypointList.clear();
		m_waypointList.push_back( cur );

		int countdown = 100;
		while( curR != r || curC != c )
		{
			if( countdown-- < 0 ) { break; }

			if( curC == c || (curR != r && rand()%2 == 0) )
			{	//Go in row direction
				int last = curR;
				if( curR < r ) { curR++; }
				else { curR--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curR = last;
					continue;
				}
			}
			else
			{	//Go in column direction
				int last = curC;
				if( curC < c ) { curC++; }
				else { curC--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curC = last;
					continue;
				}
			}

			D3DXVECTOR3 spot = g_terrain.GetCoordinates( curR, curC );
			m_waypointList.push_back( spot );
			g_terrain.SetColor( curR, curC, DEBUG_COLOR_YELLOW );
		}
		return true;
	}
}
