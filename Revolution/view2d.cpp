#include "view2d.h"
#include <QMouseEvent>
#include <iostream>

View2D::View2D()
{}

View2D::View2D(const QGLWidget* widg):
	QGLWidget(NULL,widg)
{}

void View2D::initializeGL()
{
	makeCurrent();
	glewExperimental = GL_TRUE;
	int glewErr = glewInit();
	if( glewErr != GLEW_OK )
	{
		qDebug("Error %s", glewGetErrorString(glewErr) ) ;
	}

	// la couleur de fond
	glClearColor(0.3,0.3,0.3,0.0);

	m_poly.gl_init();

	glDisable(GL_DEPTH_TEST);

}

void View2D::paintGL()
{
	makeCurrent();
	glClear(GL_COLOR_BUFFER_BIT);

	m_poly.draw(Vec3(1,1,0));

}

void View2D::mousePressEvent(QMouseEvent *event)
{
    switch(event->button())
    {
        case Qt::LeftButton:
        {
            // calcul x,y dans [-1,1]
            float x = (float(event->x()) / this->width())*2.0 - 1.0;
            float y = (float(this->height() - event->y()) / this->height())*2.0 - 1.0;
            m_poly.add_vertex(x,y);
        }
        break;
        case Qt::RightButton:
        {
            m_poly.remove_last();
        }
        break;
    }

	// MAJ affichage
	updateGL();
}

void View2D::keyPressEvent(QKeyEvent *e)
{
	switch(e->key())
	{
		case Qt::Key_L: // touche 'l'
			m_poly.lisse();
		break;

		case Qt::Key_C: // touche 'c'
			m_poly.clear();
		break;

		case Qt::Key_Escape:
			exit(EXIT_SUCCESS);
		break;

	default:
		break;
	}

	// retrace la fenetre
	updateGL();
}

void View2D::resizeGL(int width, int height)
{
	glViewport(0,0,width,height);

}
