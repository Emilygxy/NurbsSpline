#include <osg/Geometry>
#include <osgViewer/Viewer>

double N_Base_New(int i, int k, int t, double u, std::vector<float> U)//NURBS basis function
{
	//i节点向量的索引下标，k样条阶数，t样条次数，u，U，
	double N[20];//保存基函数的中间变量，大小容量最后为阶数？
	double saved, temp;
	N[0] = 1.0;
	for (int j = 1; j <= t; j++)
	{
		saved = 0.0;
		for (int r = 0; r < j; r++)//r在这里指次数,不是？
		{
			if (N[r] == 0 && (U[i + r + 1] - U[i + 1 - j + r]) == 0)
			{
				temp = 0;
			}
			else
			{
				temp = N[r] / (U[i + r + 1] - U[i + 1 - j + r]);
			}
			N[r] = saved + (U[i + r + 1] - u)*temp;
			saved = (u - U[i + 1 - j + r])*temp;

		}
		N[j] = saved;
	}
	return N[k];
}

osg::ref_ptr<osg::Vec3Array> NB_Spline_New(int t, osg::ref_ptr<osg::Vec3Array> CP, std::vector<float> U)//参数：度数，控制点，节点向量
{
	osg::ref_ptr<osg::Vec3Array> curvePs = new osg::Vec3Array;

	int Num = 50;
	auto Unumm = U.size();
	std::vector<float> NewU;
	std::vector<int> CPIndexs;
	for (int i = 0; i < Unumm; i++)
	{
		float tempu = U[i] / U[Unumm - 1];
		NewU.push_back(tempu);
	}
	for (double u = 0; u <= Num; u++)
	{
		float deltau = float(1.0) / Num*u;
		osg::Vec3 tempP(0.0, 0.0, 0.0);
		int i, j;

		if (u == Num)
		{
			auto Uvecnum = NewU.size();
			for (j = 0; j < Uvecnum - 1; j++)
			{
				if (NewU[j] <= deltau && deltau <= (NewU[j + 1]))
				{
					i = j;
					break;
				}
			}
		}
		else
		{
			auto Uvecnum = NewU.size();
			for (j = 0; j < Uvecnum - 1; j++)
			{
				if (NewU[j] <= deltau && deltau < NewU[j + 1])
				{
					i = j;
					break;
				}
			}
		}
		std::vector<osg::Vec3> tempPs;
		for (int k = 0; k <= t; k++)
		{
			osg::Vec3 tempPP = CP->at(i + k - t)*N_Base_New(i, k, t, deltau, NewU);//i:节点向量的下标索引
			tempPs.push_back(tempPP);
			CPIndexs.push_back(i + k - t);
		}
		auto Numm = tempPs.size();
		for (int k = 0; k < Numm; k++)
		{
			tempP += tempPs[k];
		}
		curvePs->push_back(tempP);
	}
	return curvePs;
}

osg::ref_ptr<osg::Group> DrawNurbsSpline()
{
	osg::ref_ptr<osg::Group> root = new osg::Group;
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Geometry> geomline = new osg::Geometry;

	//nurbspline
	osg::ref_ptr<osg::Vec3Array> nurbsconctrlP = new osg::Vec3Array;
	std::vector<float> Knots;
	std::vector<float> Weights;
	int Degree = 1;

	////1
	//Degree = 4;
	//nurbsconctrlP->push_back(osg::Vec3(43.75, -2.91276e-015, 3.24208));
	//nurbsconctrlP->push_back(osg::Vec3(42.7936, 1.81686e-028, -8.4599e-015));
	//nurbsconctrlP->push_back(osg::Vec3(40.7775, 7.3701e-015, -0.965956));
	//nurbsconctrlP->push_back(osg::Vec3(35.5301, 2.62394e-014, -4.97755));
	//nurbsconctrlP->push_back(osg::Vec3(33.168, 3.41982e-014, -9.33422));
	//nurbsconctrlP->push_back(osg::Vec3(33.168, 3.29489e-014, -15.2943));
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(1);
	//Knots.push_back(2);
	//Knots.push_back(2);
	//Knots.push_back(2);
	//Knots.push_back(2);
	//Knots.push_back(2);
	////
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);


	////2
	//Degree = 3;
	//nurbsconctrlP->push_back(osg::Vec3(1, 1, 1));
	//nurbsconctrlP->push_back(osg::Vec3(5, 10, 0));
	//nurbsconctrlP->push_back(osg::Vec3(10, 3, 0));
	//nurbsconctrlP->push_back(osg::Vec3(15, 8, 0));
	//nurbsconctrlP->push_back(osg::Vec3(17, 5, 0));
	//nurbsconctrlP->push_back(osg::Vec3(23, 8, 0));
	//nurbsconctrlP->push_back(osg::Vec3(26, 3, 0));
	//nurbsconctrlP->push_back(osg::Vec3(27, 10, 0));
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(0);
	//Knots.push_back(1.5);
	//Knots.push_back(2.5);
	//Knots.push_back(3.5);
	//Knots.push_back(4);
	//Knots.push_back(5);
	//Knots.push_back(5);
	//Knots.push_back(5);
	//Knots.push_back(5);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);
	//Weights.push_back(1);

	//3
	Degree = 4;
	nurbsconctrlP->push_back(osg::Vec3(49.5171, 3.23526e-015, 5.35743));
	nurbsconctrlP->push_back(osg::Vec3(49.5171, -4.02732e-029, 3.38396e-014));
	nurbsconctrlP->push_back(osg::Vec3(59.1107, -3.58048e-014, -5.0076));
	nurbsconctrlP->push_back(osg::Vec3(63.7476, -5.39839e-014, -8.87423));
	nurbsconctrlP->push_back(osg::Vec3(63.8262, -5.95233e-014, -17.6026));
	Knots.push_back(0);
	Knots.push_back(0);
	Knots.push_back(0);
	Knots.push_back(0);
	Knots.push_back(0);
	Knots.push_back(1);
	Knots.push_back(1);
	Knots.push_back(1);
	Knots.push_back(1);
	Knots.push_back(1);
	Weights.push_back(1);
	Weights.push_back(1);
	Weights.push_back(1);
	Weights.push_back(1);
	Weights.push_back(1);

	auto nurbsponits = NB_Spline_New(Degree, nurbsconctrlP, Knots);//
	osg::DrawElementsUInt* polyFace = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_STRIP, 0);//新建一个DrawElemnts对象(图元类型，索引数量)

	for (int i = 0; i < nurbsponits->size(); i++)
	{
		polyFace->push_back(i);
	}
	/*<绑定颜色>*/
	osg::ref_ptr<osg::Vec4Array> V_C = new osg::Vec4Array;
	V_C->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
	geom->setColorArray(V_C);
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	geom->setVertexArray(nurbsponits);
	geom->addPrimitiveSet(polyFace);
	root->addChild(geom);

	//绘制特征多边

	osg::DrawElementsUInt* polyLine = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_STRIP, 0);//新建一个DrawElemnts对象(图元类型，索引数量)
	for (int i = 0; i < nurbsconctrlP->size(); i++)
	{
		polyLine->push_back(i);
	}
	geomline->addPrimitiveSet(polyLine);
	geomline->setVertexArray(nurbsconctrlP);
	/*<绑定颜色>*/
	osg::ref_ptr<osg::Vec4Array> V_C_L = new osg::Vec4Array;
	V_C_L->push_back(osg::Vec4(0.0, 1.0, 0.0, 1.0));
	geomline->setColorArray(V_C_L);
	geomline->setColorBinding(osg::Geometry::BIND_OVERALL);
	root->addChild(geomline);

	return root;
}

int main()
{
	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root = new osg::Group;


	root->addChild(DrawNurbsSpline());


	viewer.setSceneData(root);

	return viewer.run();
}
