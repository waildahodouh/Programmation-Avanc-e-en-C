/*
 * ============================================================
 *  PROJET INDUSTRIEL - École des Sciences de l'Information
 *  Système de Détection de Collision pour Essaim Autonome (UAV)
 *  Auteur : Étudiant Ingénieur - Cycle Ingénieur Informatique
 *  Encadrant : Pr. Tarik HOUICHIME
 * ============================================================
 *
 *  ARCHITECTURE :
 *    Approche divide-and-conquer par projection sur l'axe X (tri rapide)
 *    puis balayage local (sliding window) sur les drones voisins.
 *    Complexité cible : O(n log n) au lieu de O(n²).
 *
 *  CONTRAINTE STRICTE :
 *    Aucune indexation par crochets [ ] n'est utilisée.
 *    Toute navigation se fait par arithmétique de pointeurs :
 *      *(ptr + offset)  ou  ptr->champ
 * ============================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <float.h>

// un drone avec son id et sa position dans l'espace
struct Drone {
    int   id;
    float x;
    float y;
    float z;
};

// on evite la racine carree tant qu'on compare, c'est plus rapide
static float distance_carree(const struct Drone *a, const struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

// simple echange de deux pointeurs
static void echanger(struct Drone **a, struct Drone **b) {
    struct Drone *tmp = *a;
    *a = *b;
    *b = tmp;
}

// on prend le dernier element comme pivot et on place tout ce qui est plus petit a gauche
static int partitionner(struct Drone **base, int bas, int haut) {
    float pivot_x = (*(base + haut))->x;
    int i = bas - 1;
    int j;
    for (j = bas; j < haut; j++) {
        if ((*(base + j))->x <= pivot_x) {
            i++;
            echanger(base + i, base + j);
        }
    }
    echanger(base + i + 1, base + haut);
    return i + 1;
}

// tri rapide classique, on trie par coordonnee X
static void tri_rapide(struct Drone **base, int bas, int haut) {
    if (bas < haut) {
        int pivot = partitionner(base, bas, haut);
        tri_rapide(base, bas, pivot - 1);
        tri_rapide(base, pivot + 1, haut);
    }
}

// le coeur du systeme : trouve les deux drones les plus proches
// apres le tri par X, si la difference en X seule depasse deja la meilleure
// distance connue, inutile de continuer la boucle -> gain enorme
float trouver_paire_plus_proche(struct Drone **index, int n, int *id1, int *id2) {
    float dist_min_carre = FLT_MAX;
    float dist_min = FLT_MAX;
    int i, j;

    for (i = 0; i < n - 1; i++) {
        struct Drone *di = *(index + i);
        for (j = i + 1; j < n; j++) {
            struct Drone *dj = *(index + j);

            // si juste la distance en X depasse deja notre meilleur resultat, on arrete
            float diff_x = dj->x - di->x;
            if (diff_x * diff_x >= dist_min_carre) break;

            float d2 = distance_carree(di, dj);
            if (d2 < dist_min_carre) {
                dist_min_carre = d2;
                dist_min = sqrtf(d2);  // racine carree une seule fois quand on trouve un meilleur
                *id1 = di->id;
                *id2 = dj->id;
            }
        }
    }
    return dist_min;
}

// simule les donnees qui viendraient des capteurs radar
static void generer_essaim(struct Drone *essaim, int n) {
    int i;
    for (i = 0; i < n; i++) {
        struct Drone *d = essaim + i;  // pas de crochets, on avance le pointeur
        d->id = i + 1;
        d->x = (float)(rand() % 100000) / 100.0f;  // entre 0 et 1000 metres
        d->y = (float)(rand() % 100000) / 100.0f;
        d->z = (float)(rand() % 50000)  / 100.0f;  // altitude max 500m
    }
}

// on construit un tableau de pointeurs vers les drones
// comme ca le tri ne deplace pas les drones en memoire, juste les pointeurs
static void construire_index(struct Drone *essaim, struct Drone **index, int n) {
    int i;
    for (i = 0; i < n; i++)
        *(index + i) = essaim + i;
}

int main(void) {
    const int N = 10000;

    // tout le monde dans un seul bloc memoire contigu
    struct Drone *essaim = (struct Drone *)malloc(N * sizeof(struct Drone));
    if (!essaim) {
        fprintf(stderr, "Erreur malloc essaim\n");
        return EXIT_FAILURE;
    }

    // le tableau qu'on va trier (contient des pointeurs, pas les drones eux-memes)
    struct Drone **index = (struct Drone **)malloc(N * sizeof(struct Drone *));
    if (!index) {
        fprintf(stderr, "Erreur malloc index\n");
        free(essaim);
        return EXIT_FAILURE;
    }

    srand((unsigned int)time(NULL));
    generer_essaim(essaim, N);
    construire_index(essaim, index, N);

    clock_t t_debut = clock();

    tri_rapide(index, 0, N - 1);

    int id1 = -1, id2 = -1;
    float distance = trouver_paire_plus_proche(index, N, &id1, &id2);

    double temps_ms = ((double)(clock() - t_debut) / CLOCKS_PER_SEC) * 1000.0;

    // on retrouve les pointeurs vers les deux drones detectes
    struct Drone *d1 = NULL, *d2 = NULL;
    int k;
    for (k = 0; k < N; k++) {
        struct Drone *d = essaim + k;
        if (d->id == id1) d1 = d;
        if (d->id == id2) d2 = d;
    }

    printf("Drone A : ID=%d | (%.2f, %.2f, %.2f)\n", d1->id, d1->x, d1->y, d1->z);
    printf("Drone B : ID=%d | (%.2f, %.2f, %.2f)\n", d2->id, d2->x, d2->y, d2->z);
    printf("Distance minimale : %.4f m\n", distance);
    printf("Temps : %.3f ms\n", temps_ms);

    free(index);
    free(essaim);
    return EXIT_SUCCESS;
}
