#include "meshquad.h"
#include "matrices.h"

MeshQuad::MeshQuad():
	m_nb_ind_edges(0)
{

}


void MeshQuad::gl_init()
{
	m_shader_flat = new ShaderProgramFlat();
	m_shader_color = new ShaderProgramColor();

	//VBO
	glGenBuffers(1, &m_vbo);

	//VAO
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_flat->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_flat->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &m_vao2);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_color->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_color->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);


	//EBO indices
	glGenBuffers(1, &m_ebo);
    glGenBuffers(1, &m_ebo2);
}

void MeshQuad::gl_update()
{
	//VBO
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_points.size() * sizeof(GLfloat), &(m_points[0][0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	std::vector<int> tri_indices;
	convert_quads_to_tris(m_quad_indices,tri_indices);

	//EBO indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,tri_indices.size() * sizeof(int), &(tri_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


	std::vector<int> edge_indices;
	convert_quads_to_edges(m_quad_indices,edge_indices);
	m_nb_ind_edges = edge_indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,m_nb_ind_edges * sizeof(int), &(edge_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void MeshQuad::set_matrices(const Mat4& view, const Mat4& projection)
{
	viewMatrix = view;
	projectionMatrix = projection;
}

void MeshQuad::draw(const Vec3& color)
{

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	m_shader_flat->startUseProgram();
	m_shader_flat->sendViewMatrix(viewMatrix);
	m_shader_flat->sendProjectionMatrix(projectionMatrix);
	glUniform3fv(m_shader_flat->idOfColorUniform, 1, glm::value_ptr(color));
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo);
	glDrawElements(GL_TRIANGLES, 3*m_quad_indices.size()/2,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_flat->stopUseProgram();

	glDisable(GL_POLYGON_OFFSET_FILL);

	m_shader_color->startUseProgram();
	m_shader_color->sendViewMatrix(viewMatrix);
	m_shader_color->sendProjectionMatrix(projectionMatrix);
	glUniform3f(m_shader_color->idOfColorUniform, 0.0f,0.0f,0.0f);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo2);
	glDrawElements(GL_LINES, m_nb_ind_edges,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_color->stopUseProgram();
}

void MeshQuad::clear()
{
	m_points.clear();
	m_quad_indices.clear();
}

int MeshQuad::add_vertex(const Vec3& P)
{
    m_points.push_back(P);
    return m_points.size()-1;
}


void MeshQuad::add_quad(int i1, int i2, int i3, int i4)
{
    m_quad_indices.push_back(i1);
    m_quad_indices.push_back(i2);
    m_quad_indices.push_back(i3);
    m_quad_indices.push_back(i4);
}

void MeshQuad::convert_quads_to_tris(const std::vector<int>& quads, std::vector<int>& tris)
{
	tris.clear();
	tris.reserve(3*quads.size()/2); // 1 quad = 4 indices -> 2 tris = 6 indices d'ou ce calcul (attention division entiere)

	// Pour chaque quad on genere 2 triangles
	// Attention a respecter l'orientation des triangles
    for(int i = 0; i < (int)quads.size(); i++){ // Un polyèdre à N sommets contient (N-2)/2 carrés
        tris.push_back(quads[i*4]);
        tris.push_back(quads[i*4+1]);
        tris.push_back(quads[i*4+2]);

        tris.push_back(quads[i*4]);
        tris.push_back(quads[i*4+2]);
        tris.push_back(quads[i*4+3]);
    }
}

void MeshQuad::convert_quads_to_edges(const std::vector<int>& quads, std::vector<int>& edges)
{

    auto alreadyStored = [&] (int i) -> bool
    {
         int idx = (i+1)%4;

         for(int j = 0; j < (int)edges.size(); j++){
             if(edges[j] == quads[idx] && edges[j+1] == quads[i])
                 return true;
         }
         return false;
    };

	edges.clear();
	edges.reserve(quads.size()); // ( *2 /2 !)

	// Pour chaque quad on genere 4 aretes, 1 arete = 2 indices.
	// Mais chaque arete est commune a 2 quads voisins !
	// Comment n'avoir qu'une seule fois chaque arete ?

    for(int i = 0; i < (int)quads.size(); i+=4){
        //On vérifie que l'arête inverse n'est pas déjà présente
        // Avant de l'ajouter à edges

        if(!alreadyStored(i)){
            edges.push_back(quads[i]);
            edges.push_back(quads[i+1]);
        }

        if(!alreadyStored(i+1)){
            edges.push_back(quads[i+1]);
            edges.push_back(quads[i+2]);
        }

        if(!alreadyStored(i+2)){
            edges.push_back(quads[i+2]);
            edges.push_back(quads[i+3]);
        }

        if(!alreadyStored(i+3)){
            edges.push_back(quads[i+3]);
            edges.push_back(quads[i]);
        }
    }

}


void MeshQuad::create_cube()
{
	clear();
	// ajouter 8 sommets (-1 +1)
    int s1 = add_vertex(Vec3(0,0,0));
    int s2 = add_vertex(Vec3(0.5,0,0));
       int s3 = add_vertex(Vec3(0.5,0.5,0));
       int s4 = add_vertex(Vec3(0,0.5,0));
       int s5 = add_vertex(Vec3(0,0,0.5));
       int s6 = add_vertex(Vec3(0.5,0,0.5));
       int s7 = add_vertex(Vec3(0.5,0.5,0.5));
       int s8 = add_vertex(Vec3(0,0.5,0.5));

       // ajouter 6 faces (sens trigo)
       add_quad(s1,s4,s3,s2);
       add_quad(s1,s2,s6,s5);
       add_quad(s2,s3,s7,s6);
       add_quad(s1,s5,s8,s4);
       add_quad(s4,s8,s7,s3);
       add_quad(s5,s6,s7,s8);

	gl_update();
}



Vec3 MeshQuad::normal_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// Attention a l'ordre des points !
	// le produit vectoriel n'est pas commutatif U ^ V = - V ^ U
	// ne pas oublier de normaliser le resultat.

    //Normale AB^AC
    Vec3 n1 = new Vec3((B.y - A.y)*(C.z - A.z) - (B.z - A.z)*(C.y - A.y),
                       (B.z - A.z)*(C.x - A.x) - (B.x - A.x)*(C.z - A.z),
                       (B.x - A.y)*(C.y - A.y) - (B.y - A.y)*(C.x - A.x));


    //Normale CD^CA
    Vec3 n2 = new Vec3((D.y - C.y)*(A.z - C.z) - (D.z - C.z)*(A.y - C.y),
                       (D.z - C.z)*(A.x - C.x) - (D.x - C.x)*(A.z - C.z),
                       (D.x - C.y)*(A.y - C.y) - (D.y - C.y)*(A.x - C.x));

    //Moyenne des 2 puis normalisation du vecteur en résultant
    Vec3 n = (n1+n2)/2;

    double norme_n = sqrt((n.x*n.x) + (n.y*n.y) + (n.z*n.z));

    Vec3 QuadNormal = new Vec3(n.x/norme_n, n.y/norme_n, n.z/norme_z);

    return QuadNormal;
}



float MeshQuad::area_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// aire du quad - aire tri + aire tri

	// aire du tri = 1/2 aire parallelogramme

	// aire parallelogramme: cf produit vectoriel

    //Aire ABC
    Vec3 ab = new Vec3(B.x-A.x, B.y-A.y, B.z-A.z);
    Vec3 bc = new Vec3(C.x-B.x, C.y-B.y, C.z-B.z);

    float aire_P1 = glm::length(glm::cross(ab,bc))/2;

    //Aire ACD
    Vec3 cd = new Vec3(D.x-C.x, D.y-C.y, D.z-C.z);
    Vec3 da = new Vec3(A.x-D.x, A.y-D.y, A.z-D.z);

    float aire_P2 = glm::length(glm::cross(cd,da))/2;

    //Aire ABCD
    return (aire_P1 + aire_P2);
}


bool MeshQuad::is_points_in_quad(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// On sait que P est dans le plan du quad.

	// P est-il au dessus des 4 plans contenant chacun la normale au quad et une arete AB/BC/CD/DA ?
	// si oui il est dans le quad

    Vec3 normale_quad = normal_of_quad(A,B,C,D);
    // Plan parallèle à la normale n du quad passant par AB
        // Calcul de la normale v du plan
    Vec3 AB = new Vec3(B.x-A.x, B.y-A.y, B.z-A.z);
    Vec3 normale_plan = glm::cross(normale_quad,AB);

        // Calcul de la distance entre le point P et le plan passant par AB -> si distance > 0, alors
        // P est au dessus du plan
    Vec3 AP = new Vec3(P.x-A.x, P.y-A.y, P.z-A.z);
    float distance = glm::dot(normale_plan, AP)/glm::length(normale_plan);

    if(distance < 0)
        return false;

    else{
        // Plan parallèle à la normale n du quad passant par BC
            // Calcul de la normale v du plan
        Vec3 BC = new Vec3(C.x-B.x, C.y-B.y, C.z-B.z);
        normale_plan = glm::cross(normale_quad,BC);

            // Calcul de la distance entre le point P et le plan passant par BC -> si distance > 0, alors
            // P est au dessus du plan
        Vec3 BP = new Vec3(P.x-B.x, P.y-B.y, P.z-B.z);
        distance = glm::dot(normale_plan, BP)/glm::length(normale_plan);

        if(distance < 0)
            return false;

        else{

            // Plan parallèle à la normale n du quad passant par CD
                // Calcul de la normale v du plan
            Vec3 CD = new Vec3(D.x-C.x, D.y-C.y, D.z-C.z);
            normale_plan = glm::cross(normale_quad,CD);

                // Calcul de la distance entre le point P et le plan passant par CD -> si distance > 0, alors
                // P est au dessus du plan
            Vec3 CP = new Vec3(P.x-C.x, P.y-C.y, P.z-C.z);
            distance = glm::dot(normale_plan, CP)/glm::length(normale_plan);

            if(distance < 0)
                return false;

            else{

                // Plan parallèle à la normale n du quad passant par DA
                    // Calcul de la normale v du plan
                Vec3 DA = new Vec3(A.x-D.x, A.y-D.y, A.z-D.z);
                normale_plan = glm::cross(normale_quad,DA);

                    // Calcul de la distance entre le point P et le plan passant par DA -> si distance > 0, alors
                    // P est au dessus du plan
                Vec3 AP = new Vec3(P.x-A.x, P.y-A.y, P.z-A.z);
                distance = glm::dot(normale_plan, AP)/glm::length(normale_plan);

                if(distance < 0)
                    return false;
            }
        }
    }

	return true;
}

bool MeshQuad::intersect_ray_quad(const Vec3& P, const Vec3& Dir, int q, Vec3& inter)
{
	// recuperation des indices de points
	// recuperation des points

	// calcul de l'equation du plan (N+d)

	// calcul de l'intersection rayon plan
	// I = P + alpha*Dir est dans le plan => calcul de alpha

	// alpha => calcul de I

	// I dans le quad ?

	return false;
}


int MeshQuad::intersected_visible(const Vec3& P, const Vec3& Dir)
{
	// on parcours tous les quads
	// on teste si il y a intersection avec le rayon
	// on garde le plus proche (de P)

	int inter = -1;

	return inter;
}


Mat4 MeshQuad::local_frame(int q)
{
	// Repere locale = Matrice de transfo avec
	// les trois premieres colones: X,Y,Z locaux
	// la derniere colonne l'origine du repere

	// ici Z = N et X = AB
	// Origine le centre de la face
	// longueur des axes : [AB]/2

	// recuperation des indices de points
	// recuperation des points

	// calcul de Z:N puis de X:arete on en deduit Y

	// calcul du centre

	// calcul de la taille

	// calcul de la matrice

	return Mat4();
}

void MeshQuad::extrude_quad(int q)
{
	// recuperation des indices de points

	// recuperation des points

	// calcul de la normale

	// calcul de la hauteur

	// calcul et ajout des 4 nouveaux points

	// on remplace le quad initial par le quad du dessu

	// on ajoute les 4 quads des cotes

	gl_update();
}


void MeshQuad::decale_quad(int q, float d)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// calcul de la normale

	// modification des points

	gl_update();
}


void MeshQuad::shrink_quad(int q, float s)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// ici  pas besoin de passer par une matrice
	// calcul du centre

	 // modification des points

	gl_update();
}


void MeshQuad::tourne_quad(int q, float a)
{
	// recuperation des indices de points

	// recuperation des (references de) points

	// generation de la matrice de transfo:
	// tourne autour du Z de la local frame
	// indice utilisation de glm::inverse()

	// Application au 4 points du quad


	gl_update();
}

