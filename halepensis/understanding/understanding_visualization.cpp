#include "understanding_visualization.hpp"

#include "scene_understanding.hpp"

#include <vtkCircularLayoutStrategy.h>
#include <vtkForceDirectedLayoutStrategy.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkIntArray.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
#include <vtkSimple2DLayoutStrategy.h>
#include <vtkGraphLayout.h>
#include <vtkGraphToPolyData.h>
#include <vtkGlyphSource2D.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include <map>

using std::map;
using boost::vertices;
using boost::tie;
using boost::edges;
using boost::source;
using boost::target;

auto vtkGraphFromSceneGraph(const SceneGraph& graph) -> vtkNew<vtkMutableDirectedGraph>
{
    vtkNew<vtkMutableDirectedGraph> g;
    vtkNew<vtkStringArray> vertex_labels;
    vertex_labels->SetNumberOfComponents(1);
    vertex_labels->SetName("VertexLabels");
    vtkNew<vtkStringArray> edge_labels;
    edge_labels->SetNumberOfComponents(1);
    edge_labels->SetName("EdgeLabels");
    
    map<VertexDesc, vtkIdType> id_map;
    
    {
        VertexIter i, end;
        for (tie(i, end) = vertices(graph); i != end; ++i) {
            auto& o = graph[*i];
            auto gv = g->AddVertex();
            vertex_labels->InsertNextValue(o.id);
            id_map[gv] = *i;
        }
    }
    
    {
        EdgeIter i, end;
        for (tie(i, end) = edges(graph); i != end; ++i) {
            auto& r = graph[*i];
            auto src = source(*i, graph);
            auto trg = target(*i, graph);
            g->AddEdge(id_map[src], id_map[trg]);
            edge_labels->InsertNextValue(r.description());
        }
    }
    
    g->GetVertexData()->AddArray(vertex_labels);
    g->GetEdgeData()->AddArray(edge_labels);
    
    return g;
}

auto view(const SceneUnderstanding& scene) -> void
{
    vtkNew<vtkNamedColors> colors;
    
    const auto g = vtkGraphFromSceneGraph(scene.graph);
    
    vtkNew<vtkGraphLayoutView> graphLayoutView;
    
    vtkNew<vtkGraphLayout> layout;
    vtkNew<vtkSimple2DLayoutStrategy> strategy;
    layout->SetInputData(g);
    layout->SetLayoutStrategy(strategy);
    graphLayoutView->SetLayoutStrategyToPassThrough();
    graphLayoutView->SetEdgeLayoutStrategyToPassThrough();
    graphLayoutView->AddRepresentationFromInputConnection(
          layout->GetOutputPort());
    vtkNew<vtkGraphToPolyData> graphToPoly;
    graphToPoly->SetInputConnection(layout->GetOutputPort());
    graphToPoly->EdgeGlyphOutputOn();
    graphToPoly->SetEdgeGlyphPosition(0.98);
    vtkNew<vtkGlyphSource2D> arrowSource;
    arrowSource->SetGlyphTypeToEdgeArrow();
    arrowSource->SetScale(0.1);
    arrowSource->Update();
    vtkNew<vtkGlyph3D> arrowGlyph;
    arrowGlyph->SetInputConnection(0, graphToPoly->GetOutputPort(1));
    arrowGlyph->SetInputConnection(1, arrowSource->GetOutputPort());
    vtkNew<vtkPolyDataMapper> arrowMapper;
    arrowMapper->SetInputConnection(arrowGlyph->GetOutputPort());
    vtkNew<vtkActor> arrowActor;
    arrowActor->SetMapper(arrowMapper);
    graphLayoutView->GetRenderer()->AddActor(arrowActor);
    
//    graphLayoutView->AddRepresentationFromInput(g);
//    graphLayoutView->SetLayoutStrategyToFast2D();
    
    graphLayoutView->SetVertexLabelVisibility(true);
    graphLayoutView->SetEdgeLabelVisibility(true);
    graphLayoutView->SetEdgeLabelArrayName("EdgeLabels");     // default is "labels"
    graphLayoutView->SetVertexLabelArrayName("VertexLabels"); // default is "labels"
    
    const auto graph_repr = dynamic_cast<vtkRenderedGraphRepresentation*>(graphLayoutView->GetRepresentation());
    const auto vertex_property = graph_repr->GetVertexLabelTextProperty();
    vertex_property->SetColor(colors->GetColor3d("Yellow").GetData());
    vertex_property->SetFontSize(24);
    graph_repr->SetVertexLabelTextProperty(vertex_property);
    const auto edge_property = graph_repr->GetEdgeLabelTextProperty();
    edge_property->SetColor(colors->GetColor3d("Lime").GetData());
    edge_property->SetFontSize(24);
    graph_repr->SetEdgeLabelTextProperty(edge_property);
    
    graphLayoutView->GetRenderer()->SetBackground(
                                                  colors->GetColor3d("Navy").GetData());
    graphLayoutView->GetRenderer()->SetBackground2(
                                                   colors->GetColor3d("MidnightBlue").GetData());
    graphLayoutView->GetRenderWindow()->SetWindowName("LabelVerticesAndEdges");
    graphLayoutView->ResetCamera();
    graphLayoutView->Render();
    graphLayoutView->GetInteractor()->Start();
}

auto view(const SceneUnderstanding& scene1, const SceneUnderstanding& scene2) -> void
{
    vtkNew<vtkNamedColors> colors;
    
    const auto g0 = vtkGraphFromSceneGraph(scene1.graph);
    const auto g1 = vtkGraphFromSceneGraph(scene2.graph);
    
    
    // There will be one render window
    vtkNew<vtkRenderWindow> renderWindow;
    //    renderWindow->SetSize(600, 300);
    renderWindow->SetWindowName("SideBySideGraphs");
    
    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    
    // Define viewport ranges
    // (xmin, ymin, xmax, ymax)
    double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
    double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};
    
    vtkNew<vtkGraphLayoutView> graphLayoutView0;
    graphLayoutView0->SetLayoutStrategyToForceDirected();
    graphLayoutView0->SetVertexLabelVisibility(true);
    graphLayoutView0->SetEdgeLabelVisibility(true);
    graphLayoutView0->SetEdgeLabelArrayName("EdgeLabels");     // default is "labels"
    graphLayoutView0->SetVertexLabelArrayName("VertexLabels"); // default is "labels"
    graphLayoutView0->SetRenderWindow(renderWindow);
    graphLayoutView0->SetInteractor(renderWindowInteractor);
    graphLayoutView0->GetRenderer()->SetViewport(leftViewport);
    graphLayoutView0->AddRepresentationFromInput(g0);
    
    {
        const auto graph_repr = dynamic_cast<vtkRenderedGraphRepresentation*>(graphLayoutView0->GetRepresentation());
        const auto vertex_property = graph_repr->GetVertexLabelTextProperty();
        vertex_property->SetColor(colors->GetColor3d("Yellow").GetData());
        vertex_property->SetFontSize(24);
        graph_repr->SetVertexLabelTextProperty(vertex_property);
        const auto edge_property = graph_repr->GetEdgeLabelTextProperty();
        edge_property->SetColor(colors->GetColor3d("Lime").GetData());
        edge_property->SetFontSize(24);
        graph_repr->SetEdgeLabelTextProperty(edge_property);
    }
    
    graphLayoutView0->GetRenderer()->SetBackground(
                                                   colors->GetColor3d("Navy").GetData());
    graphLayoutView0->GetRenderer()->SetBackground2(
                                                    colors->GetColor3d("MidnightBlue").GetData());
    graphLayoutView0->Render();
    graphLayoutView0->ResetCamera();
    
    vtkNew<vtkGraphLayoutView> graphLayoutView1;
    graphLayoutView1->SetLayoutStrategyToForceDirected();
    graphLayoutView1->SetVertexLabelVisibility(true);
    graphLayoutView1->SetEdgeLabelVisibility(true);
    graphLayoutView1->SetEdgeLabelArrayName("EdgeLabels");     // default is "labels"
    graphLayoutView1->SetVertexLabelArrayName("VertexLabels"); // default is "labels"
    graphLayoutView1->SetRenderWindow(renderWindow);
    graphLayoutView1->SetInteractor(renderWindowInteractor);
    graphLayoutView1->GetRenderer()->SetViewport(rightViewport);
    graphLayoutView1->AddRepresentationFromInput(g1);
    
    {
        const auto graph_repr = dynamic_cast<vtkRenderedGraphRepresentation*>(graphLayoutView1->GetRepresentation());
        const auto vertex_property = graph_repr->GetVertexLabelTextProperty();
        vertex_property->SetColor(colors->GetColor3d("Yellow").GetData());
        vertex_property->SetFontSize(24);
        graph_repr->SetVertexLabelTextProperty(vertex_property);
        const auto edge_property = graph_repr->GetEdgeLabelTextProperty();
        edge_property->SetColor(colors->GetColor3d("Lime").GetData());
        edge_property->SetFontSize(24);
        graph_repr->SetEdgeLabelTextProperty(edge_property);
    }
    
    graphLayoutView1->GetRenderer()->SetBackground(
                                                   colors->GetColor3d("DarkGreen").GetData());
    graphLayoutView1->GetRenderer()->SetBackground2(
                                                    colors->GetColor3d("ForestGreen").GetData());
    graphLayoutView1->Render();
    graphLayoutView1->ResetCamera();
    
    // graphLayoutView0->GetInteractor()->Start();
    renderWindowInteractor->Start();
}
