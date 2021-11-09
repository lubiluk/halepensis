#include "understanding_visualization.hpp"

#include "scene_understanding.hpp"

#include <vtkCircularLayoutStrategy.h>
#include <vtkForceDirectedLayoutStrategy.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkIntArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

auto vtkGraphFromSceneGraph(const SceneUnderstanding& graph) -> vtkNew<vtkMutableUndirectedGraph>
{
    vtkNew<vtkMutableUndirectedGraph> g;
    vtkNew<vtkStringArray> vertex_labels;
    vertex_labels->SetNumberOfComponents(1);
    vertex_labels->SetName("VertexLabels");
    vtkNew<vtkStringArray> edge_labels;
    edge_labels->SetNumberOfComponents(1);
    edge_labels->SetName("EdgeLabels");
    
    for (const auto& o : graph.objects)
    {
        const auto gv = g->AddVertex();
        vertex_labels->InsertNextValue(o.id);
        
        for (const auto& f : o.features) {
            const auto fv = g->AddVertex();
            vertex_labels->InsertNextValue(f->id);
            
            g->AddEdge(gv, fv);
        }
    }
    
    g->GetVertexData()->AddArray(vertex_labels);
    g->GetEdgeData()->AddArray(edge_labels);
    
    return g;
}

auto view(const SceneUnderstanding& graph) -> void
{
    vtkNew<vtkNamedColors> colors;
    
    const auto g = vtkGraphFromSceneGraph(graph);
    
    vtkNew<vtkGraphLayoutView> graphLayoutView;
    graphLayoutView->AddRepresentationFromInput(g);
    
    graphLayoutView->SetLayoutStrategyToForceDirected();
    graphLayoutView->SetVertexLabelVisibility(true);
//    graphLayoutView->SetEdgeLabelVisibility(true);
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
    
    const auto g0 = vtkGraphFromSceneGraph(scene1);
    const auto g1 = vtkGraphFromSceneGraph(scene2);
    
    
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
    //    graphLayoutView->SetEdgeLabelVisibility(true);
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
    //    graphLayoutView->SetEdgeLabelVisibility(true);
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
